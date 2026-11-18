#!/usr/bin/env bash
#
# Injects a network fault into a live NTRIP session without root privileges.
#
# A real caster will not drop your connection on request, and the usual ways of forcing it to
# (iptables, `ss --kill`) need CAP_NET_ADMIN. So this script puts a socat relay between the demo
# and the caster, and signals the relay instead: the relay's kernel then emits exactly the same
# TCP events towards the demo that a misbehaving caster or a broken mobile link would.
#
# What it cannot do is drop packets silently; a true blackhole needs the iptables procedure in
# README section 8.
#
# Usage:
#   RTK_DEMO_CASTER=ntrip://user:pass@host:port/MOUNT \
#     ./real-caster-fault.sh <demo-binary> <fault> [extra demo options ...]

set -u -o pipefail

if [ "$#" -lt 2 ]; then
    echo "usage: RTK_DEMO_CASTER=ntrip://user:pass@host:port/MOUNT $0 <demo-binary> <fault> [options]" >&2
    exit 2
fi

DEMO=$1
FAULT=$2
shift 2

if [ -z "${RTK_DEMO_CASTER:-}" ]; then
    echo "RTK_DEMO_CASTER is not set" >&2
    exit 2
fi

if [[ ! $RTK_DEMO_CASTER =~ ^ntrip://(.*)@([^:/]+):([0-9]+)(/.*)$ ]]; then
    echo "RTK_DEMO_CASTER must look like ntrip://user:pass@host:port/MOUNT" >&2
    exit 2
fi
USERINFO=${BASH_REMATCH[1]}
HOST=${BASH_REMATCH[2]}
PORT=${BASH_REMATCH[3]}
MOUNT=${BASH_REMATCH[4]}

FAULT_AT=3
for arg in "$@"; do
    [[ $arg =~ ^--fault-at=(.*)$ ]] && FAULT_AT=${BASH_REMATCH[1]}
done

# An orderly shutdown has to be a real close(), so SO_LINGER is only set for the faults that are
# supposed to produce a reset.
LINGER=",so-linger=0"
[ "$FAULT" = "fin" ] && LINGER=""

RELAY_PORT=$(python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1", 0)); print(s.getsockname()[1]); s.close()')

cleanup() {
    [ -n "${RELAY_PID:-}" ] && kill "$RELAY_PID" 2>/dev/null
    [ -n "${CHILD:-}" ] && kill -CONT "$CHILD" 2>/dev/null && kill -9 "$CHILD" 2>/dev/null
    wait 2>/dev/null
    return 0
}
trap cleanup EXIT

echo "relay: 127.0.0.1:${RELAY_PORT} -> ${HOST}:${PORT}"
socat "TCP-LISTEN:${RELAY_PORT},reuseaddr,fork,bind=127.0.0.1${LINGER}" "TCP:${HOST}:${PORT}" &
RELAY_PID=$!
sleep 0.5

RTK_DEMO_CASTER="ntrip://${USERINFO}@127.0.0.1:${RELAY_PORT}${MOUNT}" \
    "$DEMO" --fault="$FAULT" "$@" &
DEMO_PID=$!

# The demo injects nothing of its own in --caster mode, so the relay has to be hit at the same
# moment the demo records as the fault time.
sleep "$FAULT_AT"
CHILD=$(pgrep -P "$RELAY_PID" | head -1)
if [ -z "$CHILD" ]; then
    echo "!! no relay connection to disturb, is the demo connected?" >&2
else
    case $FAULT in
        reset | epipe)
            echo ">> killing the relay connection (RST)"
            kill -9 "$CHILD"
            ;;
        fin)
            echo ">> closing the relay connection (FIN)"
            kill -TERM "$CHILD"
            ;;
        stall)
            echo ">> suspending the relay, the socket stays open and goes silent"
            kill -STOP "$CHILD"
            ;;
        none)
            echo ">> no fault requested"
            ;;
        *)
            echo "!! unknown fault ${FAULT}" >&2
            ;;
    esac
fi

wait "$DEMO_PID"
exit $?
