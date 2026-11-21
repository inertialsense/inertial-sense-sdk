#!/usr/bin/env bash
#
# The silent blackhole: the link stops carrying packets without closing the connection, which is
# what a car driving into a tunnel or losing mobile coverage actually experiences. Nothing signals
# the socket, so the only way out is a kernel timeout, and that is what SO_KEEPALIVE and
# TCP_USER_TIMEOUT are for.
#
# Needs root, because silently discarding packets is what the firewall is for and there is no
# unprivileged substitute: a userspace relay cannot stop its own kernel from acknowledging data.
#
# The drop lasts long enough for the kernel to give up, then the link is restored. A build that
# noticed the outage reconnects once the link is back. A build that is wedged stays wedged, which
# is the whole point.
#
# Usage:
#   RTK_DEMO_CASTER=ntrip://user:pass@host:port/MOUNT \
#     sudo -E ./blackhole-test.sh <demo-binary> [retries2] [watchdog-ms]

set -u

if [ "$#" -lt 1 ]; then
    echo "usage: RTK_DEMO_CASTER=... sudo -E $0 <demo-binary> [retries2] [watchdog-ms]" >&2
    exit 2
fi
if [ "$(id -u)" != 0 ]; then
    echo "must run as root (sudo -E, so that RTK_DEMO_CASTER survives)" >&2
    exit 2
fi
if [ -z "${RTK_DEMO_CASTER:-}" ]; then
    echo "RTK_DEMO_CASTER is not set; use sudo -E to keep it" >&2
    exit 2
fi

DEMO=$1
# net.ipv4.tcp_retries2 is the number of retransmissions before the kernel reports ETIMEDOUT. The
# default of 15 means roughly 13-30 minutes, which is a correct but tedious test; 6 is about 50
# seconds and produces the same outcome. The patched build does not depend on this at all, because
# TCP_USER_TIMEOUT caps the wait on its own.
RETRIES2=${2:-6}
# Off by default, so that what is being measured is the port layer's own reaction to the outage
# rather than the application papering over it.
WATCHDOG=${3:-0}

if [[ ! $RTK_DEMO_CASTER =~ ^ntrip://.*@([^:/]+):([0-9]+)(/.*)$ ]]; then
    echo "RTK_DEMO_CASTER must look like ntrip://user:pass@host:port/MOUNT" >&2
    exit 2
fi
HOST=${BASH_REMATCH[1]}
PORT=${BASH_REMATCH[2]}
ADDRESS=$(getent ahostsv4 "$HOST" | awk '{print $1; exit}')
if [ -z "$ADDRESS" ]; then
    echo "cannot resolve ${HOST}" >&2
    exit 2
fi

DROP_AT=10
RESTORE_AT=90
DURATION=150

SAVED_RETRIES2=$(cat /proc/sys/net/ipv4/tcp_retries2)
RULE=(OUTPUT -p tcp -d "$ADDRESS" --dport "$PORT" -j DROP)

cleanup() {
    while iptables -C "${RULE[@]}" 2>/dev/null; do iptables -D "${RULE[@]}"; done
    sysctl -q -w "net.ipv4.tcp_retries2=${SAVED_RETRIES2}"
    echo "cleaned up: firewall rule removed, tcp_retries2 back to ${SAVED_RETRIES2}"
}
trap cleanup EXIT

echo "caster ${HOST} resolves to ${ADDRESS}:${PORT}"
echo "tcp_retries2 ${SAVED_RETRIES2} -> ${RETRIES2}"
sysctl -q -w "net.ipv4.tcp_retries2=${RETRIES2}"

# The fault is reported as `stall` so that the demo timestamps it and judges recovery for us; in
# --caster mode the demo injects nothing itself, the firewall rule below is the actual fault.
"$DEMO" --fault=stall --fault-at="$DROP_AT" --duration="$DURATION" --position-period=1 \
    --watchdog="$WATCHDOG" &
DEMO_PID=$!

sleep "$DROP_AT"
echo ">> blackholing ${ADDRESS}:${PORT}, packets are now discarded with no RST and no FIN"
iptables -I "${RULE[@]}"

sleep $((RESTORE_AT - DROP_AT))
echo ">> link restored after $((RESTORE_AT - DROP_AT))s, a healthy client reconnects from here"
iptables -D "${RULE[@]}"

wait "$DEMO_PID"
echo
echo "Read the tail of the run: corrections resuming after the restore means the build recovered."
