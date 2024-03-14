#!/usr/bin/env bash
set -o errexit

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

pushd "../python/logInspector" > /dev/null
    if [ "${CLEAN}" == "true" ]; then
        echo -e "\n\n=== Running make clean... ==="
        rm -rf build
        rm -f *.so
        rm -f *.pyc
    else
        echo -e "\n\n=== Running make... (${BUILD_TYPE}) ==="
        cd ..
        python3 -m pip install logInspector/
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
