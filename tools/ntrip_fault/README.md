# NTRIP fault-recovery tools

Manual tools for checking that an `NtripCorrectionService` recovers its correction stream after a network fault. Written by Petr Belohlavek ([petrbel](https://github.com/petrbel)) alongside PR #1388, from [his gist](https://gist.github.com/petrbel/912c104a3907fcd08474e3c9ced20b42); the files here are his, unmodified (`blackhole-test.sh` is the gist's `gistfile1.txt`).

The loopback scenarios of `rtk_reconnect_demo` run automatically in CI as `tests/test_NtripFaultRecovery.cpp`. Use these tools for what CI cannot do: a **real caster** and a **silent blackhole**, which need credentials, a public network and, for the blackhole, root.

| File | Purpose |
|---|---|
| `rtk_reconnect_demo.cpp`, `CMakeLists.txt` | The program. Starts a fake caster on loopback (or uses `--caster` / `RTK_DEMO_CASTER`), injects one fault (`reset`, `fin`, `stall`, `epipe`, `none`) and judges recovery from whether RTCM is still arriving at the END of the run. Exit status: 0 recovered, 1 did not recover, 2 bad usage or no connection, 4 nothing was streamed. |
| `real-caster-fault.sh` | Faults against a real caster through a `socat` relay, no root. Needs `socat` and `python3`. |
| `blackhole-test.sh` | Silently drops packets to a real caster with `iptables` for 80 s, with `net.ipv4.tcp_retries2` lowered. Needs root; restores the rule and the sysctl on exit. |

## Build

Point the build at an SDK you have already built (see the comments at the top of `CMakeLists.txt`):

```bash
cmake -S tools/ntrip_fault -B build-ntrip-fault \
  -DIS_SDK_INCLUDE_DIR=$PWD/src -DIS_SDK_LIBRARY=$PWD/build/libInertialSenseSDK.a
cmake --build build-ntrip-fault -j"$(nproc)"
```

Linux and glibc 2.32+ only. It needs the libusb-1.0, yaml-cpp and libudev development packages.

## Run

```bash
./build-ntrip-fault/rtk_reconnect_demo --fault=reset --fault-at=3 --duration=20     # loopback
export RTK_DEMO_CASTER='ntrip://user:password@caster.example.com:2101/MOUNT'        # keeps credentials out of ps
tools/ntrip_fault/real-caster-fault.sh ./build-ntrip-fault/rtk_reconnect_demo reset --fault-at=5 --duration=40 --position-period=1
sudo -E tools/ntrip_fault/blackhole-test.sh ./build-ntrip-fault/rtk_reconnect_demo
```

Check the `keepalive:` line printed after connecting: it shows the socket options actually set, which tells you which library you linked without injecting any fault.

## Reading the results

- Judge the final `RESULT:` line, but check the reported **longest silence**: a silence no longer than the stream's frame interval means the fault never interrupted anything.
- `stall` without `--watchdog` does not recover on any build. A caster that holds the connection open and stops sending raises no socket error; only an application-level "no corrections for N seconds" check can recover it. Gate such a check on `isReceivingCorrections()`, because `getLastRtcm3PacketAge()` is enormous before the first packet.
- `epipe` is a race between the send and receive paths. It reports the state after each write rather than asserting one outcome.
- **Blackhole:** the outage must outlast the kernel's retransmission budget, otherwise TCP just resumes and the SDK was never tested. The demo's own `RESULT:` line is not meaningful for this run; read the `opened=`/`socket=` state lines.
- The measured tables in the gist's README (section 10) were taken against an earlier version of the fix and do not all match PR #1388 as merged. In particular, the blackhole result there (the patched build never noticed the outage within 80 s) is superseded by the PR description (port closed about 65 s in, then reconnected). Re-measure before quoting them.

## Reference results, loopback (SN-9070)

The unmodified demo against `develop` just before PR #1388 and against the PR head:

| Scenario | before #1388 | with #1388 |
|---|---|---|
| `none` | 39 corrections | 39 corrections |
| `reset` (also with a 5 s watchdog) | **did not recover** (`opened=1 socket=-104`) | recovered, longest silence 0.2 s |
| `fin` | recovered | recovered |
| `stall`, 5 s watchdog | recovered | recovered |
| `stall`, no watchdog | did not recover | did not recover (expected) |
| `epipe` | **did not recover** (`valid=0`) | recovered, longest silence 0.4 s |
