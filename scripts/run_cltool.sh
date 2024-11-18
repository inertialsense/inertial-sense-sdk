#!/usr/bin/env bash
cd "$(dirname "$(realpath $0)")" > /dev/null

../cltool/build/cltool "$@"
