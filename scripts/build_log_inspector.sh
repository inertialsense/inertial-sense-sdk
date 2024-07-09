#!/usr/bin/env bash
set -o errexit

source "$(dirname "$(realpath $0)")/lib/python_venv.sh" # Load python virtual enviroment must be ran before pushd
pushd "$(dirname "$(realpath $0)")" > /dev/null

BOLD="\x1b[1m"
GREEN="\x1b[32m"
BLUE="\x1b[34m"
RESET="\x1b[0m"
CLEAN="false"
DEBUG=
BUILD_TYPE=Release

for arg in "$@"; do
    case $arg in
        -c|--clean)
            CLEAN="true"
            ;;
        -d|--debug)
            DEBUG=GDB
            BUILD_TYPE=Debug
            ;;
    esac
done

# Get the Python version
PYTHON_VERSION=$(python3 --version 2>&1)
# Extract the major and minor version numbers
PYTHON_MAJOR_VERSION=$(echo $PYTHON_VERSION | awk '{print $2}' | cut -d. -f1)
PYTHON_MINOR_VERSION=$(echo $PYTHON_VERSION | awk '{print $2}' | cut -d. -f2)
# Define your pip install command
PIP_INSTALL_COMMAND="pip3 install logInspector/"
# Check if the Python version is 3.11 or higher
if [[ $PYTHON_MAJOR_VERSION -eq 3 && $PYTHON_MINOR_VERSION -ge 11 ]] || [[ $PYTHON_MAJOR_VERSION -gt 3 ]]; then
    # If Python version is 3.11 or higher, use --break-system-packages option
    PIP_INSTALL_COMMAND="$PIP_INSTALL_COMMAND --break-system-packages"
fi

pushd "../python/logInspector" > /dev/null
    if [ "${CLEAN}" == "true" ]; then
        echo -e "\n\n=== Running make clean... ==="
        rm -rf build
        rm -f *.so
        rm -f *.pyc
    else
        echo -e "\n\n=== Running make... (${BUILD_TYPE}) ==="
        cd ..
        echo "$PIP_INSTALL_COMMAND"
        $PIP_INSTALL_COMMAND
        cd logInspector
        python3 setup.py build_ext --inplace
    fi
popd > /dev/null

timestamp="$(date)"
commit="$(git rev-parse --short HEAD)"
branch="$(git symbolic-ref --short HEAD|| echo '(detached head)')"
version="$(git describe --tags)"

# Because of `set -o errexit`, we will only get to this point if the build was successful.
# Print build information
echo -e  "${RESET}"
echo -e  "${BOLD}Log Inspector ${GREEN}Build completed on ${timestamp}${RESET}"

echo -ne "${BOLD}${CYAN}Build options: ${RESET}"
if [ "${DEBUG}" == "GDB" ]; then
    echo -ne "${BOLD}${RED}DEBUG${RESET} "
fi
if [ "${CLEAN}" == "true" ]; then
    echo -ne "${BOLD}CLEAN${RESET} "
fi
echo

echo -ne "${BOLD}${BLUE}branch: ${RESET}${branch} "
echo -ne "${BOLD}${BLUE}commit: ${RESET}${commit} "
echo -e  "${BOLD}${BLUE}version: ${RESET}${version}"
echo

popd > /dev/null
