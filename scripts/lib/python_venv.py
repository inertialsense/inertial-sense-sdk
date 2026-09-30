# python_venv.py
import os
import shutil
import sys
import venv
from typing import Optional

def is_virtual_environment(path: str) -> bool:
    # pyvenv.cfg guards against a directory that only has a stale activate script
    if not os.path.isfile(os.path.join(path, 'pyvenv.cfg')):
        return False
    if os.name == 'nt':  # Windows
        scripts_path = os.path.join(path, 'Scripts')
        return os.path.isdir(scripts_path) and os.path.isfile(os.path.join(scripts_path, 'activate.bat'))
    else:  # Unix-like
        bin_path = os.path.join(path, 'bin')
        return os.path.isdir(bin_path) and os.path.isfile(os.path.join(bin_path, 'activate'))

def create_virtual_environment(path: str) -> str:
    if is_virtual_environment(path):
        print(f"Virtual environment already exists: '{path}'.")
        return path
    if os.path.exists(path):
        if not os.path.isfile(os.path.join(path, 'pyvenv.cfg')):
            raise RuntimeError(f"'{path}' exists but is not a virtual environment. Remove it and try again.")
        # Left behind by an earlier failed venv.create() (e.g. no 'activate' script)
        print(f"Removing incomplete virtual environment: '{path}'.")
        shutil.rmtree(path)
    try:
        import ensurepip  # noqa: F401
    except ImportError:
        ver = f"{sys.version_info.major}.{sys.version_info.minor}"
        raise RuntimeError(f"Python module 'ensurepip' is missing, so a virtual environment with pip cannot be created. "
                           f"On Debian/Ubuntu run: sudo apt install python{ver}-venv")
    try:
        venv.create(path, with_pip=True)
    except Exception:
        shutil.rmtree(path, ignore_errors=True)
        raise
    print(f"New virtual environment created: '{path}'.")
    return path

def find_virtualenv() -> str:
    """
    Locate the best .venv to use for this repo layout.
    If none found, create one under <this>/../.venv (i.e., alongside SDK/scripts).
    """
    script_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))  # …/SDK/scripts
    search_roots = [
        os.getcwd(),
        script_dir,
        os.path.join(script_dir, "../../../scripts"),           # is-gpx/scripts        from is-gpx/is-common/SDK/scripts
        os.path.join(script_dir, "../../scripts"),              # is-gpx/scripts        from is-gpx/is-common/scripts
        os.path.join(script_dir, "../scripts"),                 # is-gpx/scripts        from is-gpx/scripts
        os.path.join(script_dir, "../is-common/scripts"),       # is-common/scripts     from is-gpx/scripts
        os.path.join(script_dir, "../is-common/SDK/scripts"),   # is-common/SDK/scripts from is-gpx/scripts
        os.path.join(script_dir, "../SDK/scripts"),             # is-common/SDK/scripts from is-gpx/is-common/scripts
    ]

    for root in search_roots:
        candidate = os.path.realpath(os.path.join(root, ".venv"))
        if is_virtual_environment(candidate):
            print(f"Found virtual environment: {candidate}")
            return candidate

    # Fallback: create under SDK/.venv
    create_here = os.path.realpath(os.path.join(script_dir, ".venv"))
    return create_virtual_environment(create_here)

def _site_packages_path(venv_path: str) -> Optional[str]:
    """Return the site-packages path inside venv_path, or None if not found."""
    if os.name == 'nt':  # Windows
        sp = os.path.join(venv_path, 'Lib', 'site-packages')
        return sp if os.path.isdir(sp) else None
    # Unix-like
    lib_dir = os.path.join(venv_path, 'lib')
    if not os.path.isdir(lib_dir):
        return None
    py_dirs = [d for d in os.listdir(lib_dir) if d.startswith('python')]
    if not py_dirs:
        return None
    # Select the highest version directory using version-aware comparison
    latest_py_dir = max(py_dirs, key=lambda d: tuple(map(int, d.replace('python', '').split('.'))))
    sp = os.path.join(lib_dir, latest_py_dir, 'site-packages')
    return sp if os.path.isdir(sp) else None

def activate_virtual_environment() -> bool:
    """
    "Activate" the venv for this Python process by prepending its site-packages to sys.path.
    Returns True on success, False otherwise.
    """
    venv_path = find_virtualenv()
    sp = _site_packages_path(venv_path)
    if not sp:
        print(f"Warning: site-packages not found under venv: {venv_path}")
        return False
    if sp not in sys.path:
        sys.path.insert(1, sp)
        print(f"Activated virtual environment: {venv_path}")
    return True

if __name__ == "__main__":
    # Print the resolved venv path (useful when invoked from batch/scripts)
    try:
        print(find_virtualenv())
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
