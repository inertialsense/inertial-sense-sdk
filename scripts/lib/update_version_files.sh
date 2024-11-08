#!/bin/bash
source "$(dirname "$(realpath $0)")"/../../scripts/lib/python_venv.sh #Enter VENV cause we use python here
python3 -m pip install gitpython semver
exec python3 "$(dirname "$(realpath $0)")"/update_version_files.py $@
