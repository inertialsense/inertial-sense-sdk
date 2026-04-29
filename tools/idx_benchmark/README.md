# `idx_benchmark` — v2 `.idx` perf-impact harness

Measures the storage overhead and query-time speedup the v2 `.idx`
sidecar adds on top of the `.raw` log. Drives the **full
`cISLogger` framework** with simulated-but-real-shaped telemetry
(via `tests/test_data_utils.cpp::GenerateRawLogData`), so the
numbers reflect the production write/read paths — not a microbench
of the index format in isolation.

Each row in the output corresponds to one log size; for that size
the harness records:

| column | what it measures |
|---|---|
| `raw_bytes` / `idx_bytes` | on-disk size after `cISLogger::CloseAllFiles()` |
| `write_ms` | wall-clock for the entire write phase (`InitSave` → `LogData` × N → `CloseAllFiles`) |
| `idx_full_read_ms` | time to read every record out of every `.idx` segment into memory |
| `search_via_idx_ms` | linear scan of the in-memory record vector for `did == target` |
| `search_via_raw_ms` | walk every packet via `cISLogger::ReadNextPacket`, filter for the same DID |
| `match_count` | number of matches (used as a sanity check; both methods must agree) |

The "search" workload is "find all packets with `DID == X`" where
`X` is the most-frequent DID in the stream. Filtering by additional
predicates (e.g., timestamp range) would only widen the gap.

## Building

The benchmark links against the `InertialSenseSDK` static library,
so you must build that first. From the SDK root:

```sh
# 1. Build the static lib (once per change to src/)
cd build-test && cmake .. && cmake --build . -j   # or the SDK's
                                                  # standard build dir

# 2. Build this tool
cd ../tools/idx_benchmark
mkdir -p build && cd build
cmake .. && cmake --build . -j
```

The resulting binary is `tools/idx_benchmark/build/idx_benchmark`.

## Running

Default run sweeps 16 sizes from 0.25 MB to 128 MB:

```sh
./idx_benchmark
```

CLI options:

```
--sizes <a,b,c,...>   comma-separated MB values
                      (default: 0.25,0.5,1,2,3,4,6,8,12,16,24,32,48,64,96,128)
--out-csv <path>      also write a plain CSV file (no #CSV markers)
--out-dir <dir>       scratch directory for log files (default: /tmp)
--quiet               suppress per-row progress lines on stderr
```

Output is always a CSV between `#CSV-START` / `#CSV-END` sentinels
on stdout. Per-row progress goes to stderr unless `--quiet`. Example:

```sh
./idx_benchmark --sizes 1,4,16,64 --out-csv results.csv
```

Approximate wall-clock cost on a moderate Linux VM:
- 8 MB  → ~1 s
- 32 MB → ~12 s
- 128 MB → ~50 s

The 128 MB case implicitly exercises multi-segment rotation (the
default segment cap is 5 MB → ~25 segments) and is therefore also
useful as a smoke test for the writer's segment-rotation path.

## Plotting

`plot.py` produces a 4-panel summary PNG (file size, storage
overhead %, query time on log-log, search speedup). It accepts the
benchmark's stdout directly or a CSV file.

```sh
# Pipe directly:
./idx_benchmark | python3 ../plot.py - out.png

# Or separately:
./idx_benchmark --out-csv /tmp/run.csv > /dev/null
python3 ../plot.py /tmp/run.csv out.png
```

Requires `matplotlib` and `pandas`:

```sh
pip install matplotlib pandas        # or your distro's preferred way
```

## Establishing a baseline

A snapshot from the D-01 / SN-7879 run is committed at
`docs/perf/idx-v2-baseline.{png,csv}`. To re-establish or compare:

```sh
./tools/idx_benchmark/build/idx_benchmark --out-csv \
    docs/perf/idx-v2-baseline.csv > /tmp/bench.out
python3 tools/idx_benchmark/plot.py /tmp/bench.out \
    docs/perf/idx-v2-baseline.png
```

If you're investigating a regression, compare your run's CSV row
with the committed baseline's same-size row — particularly
`search_via_idx_ms` and `idx_bytes / raw_bytes`.

## See also

- `src/ISLogIndex.{h,cpp}` — the v2 format definition + parse helpers
- `src/DeviceLog.cpp` — `addIndexRecord` / `writeIndexChunk` / `finalizeIndex`
- `src/DeviceLogRaw.cpp` — per-packet `addIndexRecord` call site
- `tests/test_log_index.cpp` — correctness tests for the same code
