# python_venv.py
import glob
import os
import shutil
import subprocess
import sys
import venv
from typing import Optional

def is_virtual_environment(path: str) -> bool:
    # pyvenv.cfg guards against a directory that only has a stale activate script
    if not os.path.isfile(os.path.join(path, 'pyvenv.cfg')):
        return False
    # pip is checked too: activate is written before pip is installed, so an interrupted setup has activate but no pip
    if os.name == 'nt':  # Windows
        scripts_path = os.path.join(path, 'Scripts')
        return os.path.isfile(os.path.join(scripts_path, 'activate.bat')) and os.path.isfile(os.path.join(scripts_path, 'pip.exe'))
    else:  # Unix-like
        bin_path = os.path.join(path, 'bin')
        return os.path.isfile(os.path.join(bin_path, 'activate')) and os.path.isfile(os.path.join(bin_path, 'pip'))

def _venv_python(path: str) -> str:
    if os.name == 'nt':
        return os.path.join(path, 'Scripts', 'python.exe')
    return os.path.join(path, 'bin', 'python')

def _bootstrap_pip(path: str) -> None:
    """
    Install pip into a venv created without it. Tries the distro's bundled pip wheel first (offline),
    then the system pip targeting the venv. Output goes to stderr: callers read the venv path from stdout.
    """
    venv_python = _venv_python(path)
    attempts = []
    # Debian/Ubuntu python3-pip-whl; a pip wheel can run itself to install itself
    for wheel in sorted(glob.glob('/usr/share/python-wheels/pip-*.whl'), reverse=True):
        attempts.append(("bundled pip wheel", [venv_python, os.path.join(wheel, 'pip'), 'install', '--no-index', '--quiet', wheel]))
    # System pip (python3-pip) installing into the venv's interpreter; PEP 668 doesn't apply to the venv
    attempts.append(("system pip", [sys.executable, '-m', 'pip', '--python', venv_python, 'install', '--quiet', 'pip']))

    for name, cmd in attempts:
        print(f"Installing pip into virtual environment using {name}...", file=sys.stderr)
        if subprocess.run(cmd, stdout=sys.stderr).returncode == 0:
            return

    ver = f"{sys.version_info.major}.{sys.version_info.minor}"
    raise RuntimeError(f"Could not install pip into the virtual environment: Python module 'ensurepip' is missing and "
                       f"pip is not available. On Debian/Ubuntu run: sudo apt install python{ver}-venv (or python3-pip)")

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
        has_ensurepip = True
    except ImportError:
        # Debian/Ubuntu ship ensurepip separately (python3-venv); create without pip and add it ourselves
        has_ensurepip = False
    try:
        venv.create(path, with_pip=has_ensurepip)
        if not has_ensurepip:
            _bootstrap_pip(path)
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
