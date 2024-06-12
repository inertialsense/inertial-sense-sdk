SCRIPT_DIR="$(dirname "$(realpath $0)")"
if [ -n "${VIRTUAL_ENV+x}" ]; then
	echo "Virtual Enviroment already activated"
	echo "VENV: $VIRTUAL_ENV"
elif [ -r $PWD/.venv/bin/activate ]; then
	source $PWD/.venv/bin/activate
	echo "Activated Virtual Enviroment at $PWD/.venv"
elif [ -r $SCRIPT_DIR/.venv/bin/activate ]; then
	source $SCRIPT_DIR/.venv/bin/activate
	echo "Activated Virtual Enviroment at $SCRIPT_DIR/.venv"
else
	echo "No Virtual Enviroment found creating one at $SCRIPT_DIR/.venv"
	pushd $SCRIPT_DIR > /dev/null
	python3 -m venv .venv
	popd > /dev/null
	source $SCRIPT_DIR/.venv/bin/activate
	echo "Activated Virtual Enviroment at $SCRIPT_DIR/.venv"
fi
