"""
generate_doxygen.py -- Generate Doxygen documentation for the Inertial Sense SDK.

Usage:
    python3 scripts/generate_doxygen.py [-o OUTPUT_DIR] [--pdf]

Options:
    -o OUTPUT_DIR   Directory where docs are written (default: <repo>/docs)
    --pdf           Also generate a PDF via LaTeX (requires pdflatex)
    -h, --help      Show this message and exit

The script resolves the repository root from its own location, then injects
runtime settings by piping docs/Doxyfile together with override lines into
`doxygen -`.  The committed docs/Doxyfile is never modified.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DOXYFILE = REPO_ROOT / "docs" / "Doxyfile"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate Doxygen HTML (and optionally PDF) docs for the IS SDK.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "-o", "--output-dir",
        default=str(REPO_ROOT / "docs"),
        metavar="OUTPUT_DIR",
        help="Directory where docs are written (default: %(default)s)",
    )
    parser.add_argument(
        "--pdf",
        action="store_true",
        help="Also generate a PDF via LaTeX (requires pdflatex in PATH)",
    )
    return parser.parse_args()


def check_dep(name: str, install_hint: str) -> None:
    """Exit with an error if *name* is not found on PATH."""
    if shutil.which(name) is None:
        print(f"ERROR: '{name}' not found in PATH.\n  Install with: {install_hint}",
              file=sys.stderr)
        sys.exit(1)


def detect_version() -> str:
    """Return the most recent git tag, or an empty string if unavailable."""
    try:
        result = subprocess.run(
            ["git", "-C", str(REPO_ROOT), "describe", "--tags", "--abbrev=0"],
            capture_output=True, text=True,
        )
        return result.stdout.strip() if result.returncode == 0 else ""
    except FileNotFoundError:
        return ""


def run() -> None:
    args = parse_args()

    # Dependency checks
    check_dep("doxygen",
              "Linux: sudo apt-get install -y doxygen  |  "
              "macOS: brew install doxygen  |  "
              "Windows: choco install doxygen.install")
    if args.pdf:
        check_dep("pdflatex",
                  "Linux: sudo apt-get install -y texlive-latex-recommended  |  "
                  "macOS: brew install --cask mactex-no-gui")

    # Resolve output directory (create if needed)
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    # SDK source INPUT list — only src/; cltool and ExampleProjects are excluded
    src_dir = REPO_ROOT / "src"
    if not src_dir.is_dir():
        print(f"ERROR: SDK src/ directory not found: {src_dir}", file=sys.stderr)
        sys.exit(1)

    # USE_MDFILE_AS_MAINPAGE requires the README to be in the INPUT list
    readme = REPO_ROOT / "README.md"
    input_dirs = str(src_dir)
    mainpage_override = ""
    if readme.is_file():
        input_dirs = f"{src_dir} {readme}"
        mainpage_override = f"USE_MDFILE_AS_MAINPAGE = {readme}"

    version = detect_version()

    # Build the combined Doxyfile content (base + runtime overrides)
    overrides = [
        "",
        "# --- Runtime overrides injected by generate_doxygen.py ---",
        f"OUTPUT_DIRECTORY = {output_dir}",
        f"INPUT            = {input_dirs}",
        f"GENERATE_LATEX   = {'YES' if args.pdf else 'NO'}",
    ]
    if version:
        overrides.append(f"PROJECT_NUMBER   = {version}")
    if mainpage_override:
        overrides.append(mainpage_override)

    combined = DOXYFILE.read_text(encoding="utf-8") + "\n".join(overrides) + "\n"

    # Status
    print("Generating Inertial Sense SDK documentation...")
    print(f"  Repo root  : {REPO_ROOT}")
    print(f"  Source     : {input_dirs}")
    print(f"  Output dir : {output_dir}")
    print(f"  Version    : {version or '<not detected>'}")
    print(f"  PDF        : {args.pdf}")
    print()

    subprocess.run(["doxygen", "-"], input=combined, text=True, check=True)

    print()
    print(f"HTML documentation written to: {output_dir / 'html' / 'index.html'}")

    # Optional PDF build
    if args.pdf:
        latex_dir = output_dir / "latex"
        print()
        print(f"Building PDF from LaTeX sources in: {latex_dir}")
        if not latex_dir.is_dir():
            print(f"ERROR: LaTeX output directory not found: {latex_dir}", file=sys.stderr)
            sys.exit(1)
        subprocess.run(["make"], cwd=latex_dir, check=True)
        pdf_path = latex_dir / "refman.pdf"
        if pdf_path.is_file():
            print(f"PDF written to: {pdf_path}")
        else:
            print(f"ERROR: PDF build finished but refman.pdf not found in {latex_dir}",
                  file=sys.stderr)
            sys.exit(1)

    print()
    print("Done.")


if __name__ == "__main__":
    run()
