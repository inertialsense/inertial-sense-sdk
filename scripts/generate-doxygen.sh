#!/usr/bin/env bash
# =============================================================================
# generate-doxygen.sh
#
# Description:
#   Generates Doxygen HTML (and optionally PDF) documentation for the
#   Inertial Sense SDK.  The script detects the repository root, resolves
#   absolute source paths, and pipes runtime overrides into `doxygen -` so
#   the base scripts/Doxyfile is never modified.
#
# Usage:
#   bash scripts/generate-doxygen.sh [OPTIONS]
#
# Options:
#   -o <output_dir>   Directory where docs are written (default: ./docs)
#   --pdf             Also generate a PDF via LaTeX (requires pdflatex)
#   -h, --help        Show this help message and exit
#
# Examples:
#   bash scripts/generate-doxygen.sh
#   bash scripts/generate-doxygen.sh -o ./build/docs
#   bash scripts/generate-doxygen.sh -o ./build/docs --pdf
#
# Notes:
#   - Run from any directory; the script resolves paths relative to itself.
#   - chmod +x this file after cloning if needed:
#       git update-index --chmod=+x scripts/generate-doxygen.sh
# =============================================================================

set -euo pipefail

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

usage() {
    sed -n '/^# Usage:/,/^# ====/p' "$0" | grep '^#' | sed 's/^# \?//'
    exit 0
}

die() {
    echo "ERROR: $*" >&2
    exit 1
}

# ---------------------------------------------------------------------------
# Resolve paths
# ---------------------------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DOXYFILE="${SCRIPT_DIR}/Doxyfile"

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------

OUTPUT_DIR="${REPO_ROOT}/docs"
GENERATE_PDF=false

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

while [[ $# -gt 0 ]]; do
    case "$1" in
        -o)
            [[ -n "${2:-}" ]] || die "-o requires an argument"
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --pdf)
            GENERATE_PDF=true
            shift
            ;;
        -h|--help)
            usage
            ;;
        *)
            die "Unknown option: $1  (use -h for help)"
            ;;
    esac
done

# ---------------------------------------------------------------------------
# Dependency checks
# ---------------------------------------------------------------------------

if ! command -v doxygen &>/dev/null; then
    die "doxygen not found in PATH.
  Install with:
    Linux:  sudo apt-get install -y doxygen
    macOS:  brew install doxygen
    Windows: choco install doxygen.install"
fi

if $GENERATE_PDF; then
    if ! command -v pdflatex &>/dev/null; then
        die "pdflatex not found in PATH (required for --pdf).
  Install with:
    Linux:  sudo apt-get install -y texlive-latex-base texlive-latex-recommended
    macOS:  brew install --cask mactex-no-gui"
    fi
fi

# ---------------------------------------------------------------------------
# Resolve output directory to absolute path
# ---------------------------------------------------------------------------

# Create early so we can resolve with realpath-style expansion
mkdir -p "${OUTPUT_DIR}"
OUTPUT_DIR="$(cd "${OUTPUT_DIR}" && pwd)"

# ---------------------------------------------------------------------------
# Determine version (best-effort; empty string is fine)
# ---------------------------------------------------------------------------

VERSION="$(git -C "${REPO_ROOT}" describe --tags --abbrev=0 2>/dev/null || true)"

# ---------------------------------------------------------------------------
# Build source INPUT list (absolute paths, space-separated for Doxygen)
# ---------------------------------------------------------------------------

INPUT_DIRS=""
for rel_dir in src cltool/src; do
    abs_dir="${REPO_ROOT}/${rel_dir}"
    if [[ -d "${abs_dir}" ]]; then
        INPUT_DIRS="${INPUT_DIRS} ${abs_dir}"
    else
        echo "WARNING: source directory not found, skipping: ${abs_dir}" >&2
    fi
done
INPUT_DIRS="${INPUT_DIRS# }"   # strip leading space

[[ -n "${INPUT_DIRS}" ]] || die "No source directories found under ${REPO_ROOT}"

# ---------------------------------------------------------------------------
# Compute USE_MDFILE_AS_MAINPAGE (only if README.md exists at repo root)
# ---------------------------------------------------------------------------

MAINPAGE_OVERRIDE=""
if [[ -f "${REPO_ROOT}/README.md" ]]; then
    MAINPAGE_OVERRIDE="USE_MDFILE_AS_MAINPAGE = ${REPO_ROOT}/README.md"
fi

# ---------------------------------------------------------------------------
# Set LaTeX flag
# ---------------------------------------------------------------------------

if $GENERATE_PDF; then
    LATEX_FLAG="YES"
else
    LATEX_FLAG="NO"
fi

# ---------------------------------------------------------------------------
# Run Doxygen
# ---------------------------------------------------------------------------

echo "Generating Inertial Sense SDK documentation..."
echo "  Repo root   : ${REPO_ROOT}"
echo "  Source dirs : ${INPUT_DIRS}"
echo "  Output dir  : ${OUTPUT_DIR}"
echo "  Version     : ${VERSION:-<not detected>}"
echo "  PDF         : ${GENERATE_PDF}"
echo ""

(
    cat "${DOXYFILE}"
    echo ""
    echo "# --- Runtime overrides injected by generate-doxygen.sh ---"
    echo "OUTPUT_DIRECTORY       = ${OUTPUT_DIR}"
    echo "INPUT                  = ${INPUT_DIRS}"
    echo "GENERATE_LATEX         = ${LATEX_FLAG}"
    [[ -n "${VERSION}" ]] && echo "PROJECT_NUMBER         = ${VERSION}" || true
    [[ -n "${MAINPAGE_OVERRIDE}" ]] && echo "${MAINPAGE_OVERRIDE}" || true
) | doxygen -

echo ""
echo "HTML documentation written to: ${OUTPUT_DIR}/html/index.html"

# ---------------------------------------------------------------------------
# Optional PDF build
# ---------------------------------------------------------------------------

if $GENERATE_PDF; then
    LATEX_DIR="${OUTPUT_DIR}/latex"
    echo ""
    echo "Building PDF from LaTeX sources in: ${LATEX_DIR}"
    if [[ ! -d "${LATEX_DIR}" ]]; then
        die "LaTeX output directory not found: ${LATEX_DIR}"
    fi
    (cd "${LATEX_DIR}" && make 2>&1)
    PDF_PATH="${LATEX_DIR}/refman.pdf"
    if [[ -f "${PDF_PATH}" ]]; then
        echo "PDF written to: ${PDF_PATH}"
    else
        die "PDF build finished but refman.pdf not found in ${LATEX_DIR}"
    fi
fi

echo ""
echo "Done."
