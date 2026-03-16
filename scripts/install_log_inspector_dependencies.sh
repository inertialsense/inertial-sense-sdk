#!/bin/bash
cd "$(dirname "$(realpath $0)")" > /dev/null
source lib/activate_python_venv.sh

if [[ "$(uname -s)" == "Darwin" ]]; then
    # macOS prerequisites for WeasyPrint (used by mkdocs-with-pdf)
    if command -v brew >/dev/null 2>&1; then
        brew install cairo pango gdk-pixbuf libffi libxml2 libxslt gettext || true
    else
        echo "Homebrew not found; install it to fetch WeasyPrint native deps."
    fi
else
    sudo apt install -y python3 python3-pip \
        libcairo2 libpango-1.0-0 libpangocairo-1.0-0 libgdk-pixbuf2.0-0 libffi-dev libxml2 libxslt1.1 fonts-liberation shared-mime-info
fi

python3 -m pip install -U pip # update pip3 to latest version
python3 -m pip install setuptools wheel
python3 -m pip install pytest pybind11

./build_log_inspector.sh

source ~/.bashrc
