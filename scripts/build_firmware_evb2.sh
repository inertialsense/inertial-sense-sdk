#!/usr/bin/env bash
set -o errexit
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
pushd "$(dirname "$(realpath $0)")" > /dev/null

BOLD="\x1b[1m"
GREEN="\x1b[32m"
BLUE="\x1b[34m"
RESET="\x1b[0m"

DEBUG=

for i in "$@"; do
    case "$1" in
        -c|--clean)
            clean="true"
            shift
            ;;
        -d|--debug)
            DEBUG=GDB
            shift
            ;;
    esac
done

pushd "../EVB-2" > /dev/null
    if [ -n "${clean}" ]; then
        echo -e "\n\n=== Running make clean... ==="
        make clean
    else
        echo -e "\n\n=== Running make... ==="
        make -j`nproc` DEBUG=${DEBUG}

        set +o errexit
        source $SCRIPT_DIR/lib/release_append_header_version.sh ../../EVB-2/Release/IS_EVB-2.hex ../../EVB-2/IS_EVB-2/src $1
        set -o errexit
    fi

popd > /dev/null

commit="$(git rev-parse --short HEAD)"
branch="$(git symbolic-ref --short HEAD|| echo '(detached head)')"
version="$(git describe --tags)"

# Because of `set -o errexit`, we will only get to this point if the build was successful.
# Print build information
echo -e  "${RESET}"
echo -e  "${BOLD}EVB-2 ${GREEN}Firmware build complete on ${BUILD_DATE_FORMAT} ${BUILD_TIME_FORMAT}${RESET}"

echo -ne "${BOLD}${CYAN}Build options: ${RESET}"
if [ "${DEBUG}" == "GDB" ]; then
    echo -ne "${BOLD}${RED}DEBUG${RESET} "
fi
if [ -n "${clean}" ]; then
    echo -ne "${BOLD}CLEAN${RESET} "
fi
echo

echo -ne "${BOLD}${BLUE}branch: ${RESET}${branch} "
echo -ne "${BOLD}${BLUE}commit: ${RESET}${commit} "
echo -e  "${BOLD}${BLUE}version: ${RESET}${version}"
echo -ne  "${BOLD}${GREEN}Firmware path: ${RESET}"
echo -e "${BOLD}"$(realpath ../EVB-2/obj/EVB-2.hex)"${RESET}"
echo

popd > /dev/null
