#!/usr/bin/env bash

python2 -m pip install --upgrade pip
python2 -m pip install --upgrade build
python2 -m build
python2 -m pip install --upgrade twine
python2 -m twine upload --verbose dist/*
python2 -m pip install --no-deps inertialsense_math
