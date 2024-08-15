# This script sets the python virtual environment based on the first .venv directory found, either in $PWD or in DIR_SEARCH_LIST.

SCRIPT_DIR="$(dirname "$(realpath $0)")"
DIR_SEARCH_LIST=(
    $PWD,
    $SCRIPT_DIR,
    $SCRIPT_DIR/../../../scripts,           # finds is-gpx/scripts from is-gpx/is-common/SDK/scripts
    $SCRIPT_DIR/../../scripts,              # finds is-gpx/scripts from is-gpx/is-common/scripts
    $SCRIPT_DIR/../scripts,                 # finds is-gpx/scripts from is-gpx/scripts
    $SCRIPT_DIR/../is-common/scripts,       # from is-gpx/scripts finds is-common/scripts 
    $SCRIPT_DIR/../is-common/SDK/scripts,   # from is-gpx/scripts finds is-common/SDK/scripts
    $SCRIPT_DIR/../SDK/scripts              # from is-gpx/is-common/scripts finds is-common/SDK/scripts
)

if [ -n "${VIRTUAL_ENV+x}" ]; then
    echo "Virtual Enviroment already activated"
    echo "VENV: $VIRTUAL_ENV"
else
    for directory in "${DIR_SEARCH_LIST[@]}"
    do	# Search directory list for first .venv directory that exists
        # echo "Checking directory: ${directory}"
        if [ -r  $directory/.venv/bin/activate ]; then
            source $directory/.venv/bin/activate
            echo "Activated Virtual Enviroment at $(realpath $directory/.venv)"
            break
        fi
    done

    if [ -z "${VIRTUAL_ENV+x}" ]; then
        echo "No Virtual Enviroment found creating one at $SCRIPT_DIR/.venv"
        pushd $SCRIPT_DIR > /dev/null
        python3 -m venv .venv
        popd > /dev/null
        source $SCRIPT_DIR/.venv/bin/activate
        echo "Activated Virtual Enviroment at $SCRIPT_DIR/.venv"
    fi
fi
