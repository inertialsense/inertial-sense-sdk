#!/usr/bin/env python3
"""Plot v2 .idx benchmark results.

Reads CSV from stdin or a file argument. The CSV may be either bare
(11 columns starting with `size_mb,...`) or wrapped between
`#CSV-START` / `#CSV-END` sentinels — the format the benchmark
binary writes to stdout.

Usage:
    idx_benchmark | tools/idx_benchmark/plot.py - out.png
    tools/idx_benchmark/plot.py results.csv out.png

Requires: matplotlib, pandas (`pip install matplotlib pandas`).
"""
import io
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd


def load_csv(text: str) -> pd.DataFrame:
    m = re.search(r"#CSV-START\n(.*?)\n#CSV-END", text, re.DOTALL)
    body = m.group(1) if m else text
    return pd.read_csv(io.StringIO(body))


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__, file=sys.stderr)
        return 1
    src_arg, out_path = sys.argv[1], Path(sys.argv[2])
    text = sys.stdin.read() if src_arg == "-" else Path(src_arg).read_text()

    df = load_csv(text)
    raw_mb = df["raw_bytes"] / (1024 * 1024)
    idx_mb = df["idx_bytes"] / (1024 * 1024)
    pct = idx_mb / raw_mb * 100
    speedup = df["search_via_raw_ms"] / df["search_via_idx_ms"]

    fig, axes = plt.subplots(2, 2, figsize=(13.5, 9))
    fig.suptitle(
        "v2 .idx impact analysis  —  cISLogger end-to-end with simulated telemetry",
        fontsize=14, y=0.995,
    )

    # File size: raw vs idx
    ax = axes[0, 0]
    ax.plot(raw_mb, raw_mb, marker="o", linewidth=2, color="#4C72B0", label=".raw")
    ax.plot(raw_mb, idx_mb, marker="s", linewidth=2, color="#DD8452", label=".idx")
    ax.set_xscale("log"); ax.set_yscale("log")
    ax.set_xlabel(".raw size on disk (MB, log scale)")
    ax.set_ylabel("file size (MB, log scale)")
    ax.set_title("File size on disk")
    ax.legend(); ax.grid(alpha=0.3, which="both")

    # Storage overhead %
    ax = axes[0, 1]
    ax.plot(raw_mb, pct, marker="o", color="#C44E52", linewidth=2)
    ax.set_xscale("log")
    ax.set_xlabel(".raw size on disk (MB, log scale)")
    ax.set_ylabel(".idx as % of .raw")
    ax.set_title("Storage overhead")
    ax.grid(alpha=0.3, which="both")
    ax.set_ylim(0, max(pct) * 1.4)
    ax.axhline(pct.mean(), linestyle="--", color="#888", alpha=0.6,
               label=f"mean = {pct.mean():.2f}%")
    ax.legend()

    # Query time, log-log
    ax = axes[1, 0]
    ax.plot(raw_mb, df["search_via_raw_ms"], marker="o", linewidth=2,
            label=".raw scan (cISLogger::ReadNextPacket)", color="#C44E52")
    ax.plot(raw_mb, df["search_via_idx_ms"], marker="s", linewidth=2,
            label=".idx record scan", color="#4C72B0")
    ax.set_xscale("log"); ax.set_yscale("log")
    ax.set_xlabel(".raw size on disk (MB, log scale)")
    ax.set_ylabel('"find all packets with DID==X"  query time (ms, log scale)')
    ax.set_title("Query time — same workload, two methods")
    ax.grid(alpha=0.3, which="both")
    ax.legend()

    # Speedup factor
    ax = axes[1, 1]
    ax.plot(raw_mb, speedup, marker="^", color="#8172B2", linewidth=2,
            markersize=7)
    ax.set_xscale("log")
    ax.set_xlabel("log size (MB, log scale)")
    ax.set_ylabel("speedup (×)")
    ax.set_title(".idx-based search speedup vs full .raw scan")
    ax.grid(alpha=0.3, which="both")
    ax.axhline(speedup.mean(), linestyle="--", color="#666", alpha=0.6,
               label=f"mean = {speedup.mean():.0f}×")
    ax.legend()
    for i in (0, len(df) - 1):
        ax.annotate(f"{speedup.iloc[i]:.0f}×",
                    (raw_mb.iloc[i], speedup.iloc[i]),
                    textcoords="offset points", xytext=(0, 8),
                    ha="center", fontsize=9)

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out_path, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out_path} ({len(df)} data points)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
