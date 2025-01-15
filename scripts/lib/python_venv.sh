# This script must be sourced (not executed) to setup python virtual environment.
# The python virtual environment is set based on the first .venv directory found, either in $PWD or in DIR_SEARCH_LIST.

SCRIPT_DIR=$(dirname "$(dirname "$(realpath "${BASH_SOURCE[0]}")")")
DIR_SEARCH_LIST=(
    $PWD
    $SCRIPT_DIR
    $SCRIPT_DIR/../../../scripts            # finds is-gpx/scripts from is-gpx/is-common/SDK/scripts
    $SCRIPT_DIR/../../scripts               # finds is-gpx/scripts from is-gpx/is-common/scripts
    $SCRIPT_DIR/../scripts                  # finds is-gpx/scripts from is-gpx/scripts
    $SCRIPT_DIR/../is-common/scripts        # from is-gpx/scripts finds is-common/scripts 
    $SCRIPT_DIR/../is-common/SDK/scripts    # from is-gpx/scripts finds is-common/SDK/scripts
    $SCRIPT_DIR/../SDK/scripts              # from is-gpx/is-common/scripts finds is-common/SDK/scripts
)

if [ -n "${VIRTUAL_ENV+x}" ]; then
    echo "Virtual environment already activated: $VIRTUAL_ENV"
else
    for directory in "${DIR_SEARCH_LIST[@]}"
    do	# Search directory list for first .venv directory that exists
        # echo "Checking directory: ${directory}"
        if [ -r  "$directory/.venv/bin/activate" ]; then
            source $directory/.venv/bin/activate
            echo "Activated virtual environment: $(realpath $directory/.venv)"
            break
        fi
    done

    if [ -z "${VIRTUAL_ENV+x}" ]; then
        echo "No virtual environment found, creating one: $SCRIPT_DIR/.venv"
        pushd $SCRIPT_DIR > /dev/null
        python3 -m venv .venv
        popd > /dev/null
        source $SCRIPT_DIR/.venv/bin/activate
        echo "Activated virtual environment: $SCRIPT_DIR/.venv"
    fi
fi
