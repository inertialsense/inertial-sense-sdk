# This script must be sourced (not executed) to setup python virtual environment.
# Kept for existing callers; finding/creating the .venv is done by activate_python_venv.sh (python_venv.py),
# which searches the same directories and can create the venv even when ensurepip (python3-venv) is missing.

if [ -n "${VIRTUAL_ENV+x}" ]; then
    echo "Virtual environment already activated: $VIRTUAL_ENV"
else
    source "$(dirname "$(realpath "${BASH_SOURCE[0]}")")/activate_python_venv.sh"
fi
