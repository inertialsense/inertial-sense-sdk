#!/usr/bin/env bash

python3 -m pip install --upgrade pip
python3 -m pip install --upgrade build
python3 -m build
python3 -m pip install --upgrade twine
python3 -m twine upload --verbose dist/*
python3 -m pip install --no-deps inertialsense_math
