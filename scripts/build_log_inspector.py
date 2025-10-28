from __future__ import annotations
import os
import sys
import platform
import subprocess
import shutil
import pathlib
import runpy
import ctypes, importlib
from build_manager import BuildTestManager
from contextlib import contextmanager

# ---------- Paths & environment ----------
SDK_DIR = pathlib.Path(__file__).resolve().parent.parent
PYTHON_DIR = SDK_DIR / "python"
IS_WINDOWS = (os.name == "nt") or (platform.system() == "Windows")
PY = sys.executable  # venv python when activated

def in_venv() -> bool:
    return sys.prefix != getattr(sys, "base_prefix", sys.prefix)

_DLL_DIR_HANDLES: list = []
_DLLS_ADDED = False

def _add_sdk_dll_dirs(verbose: bool = True) -> None:
    global _DLLS_ADDED
    if _DLLS_ADDED or os.name != "nt":
        return
    added, seen = [], set()

    def add_dir(p: pathlib.Path | str | None):
        if not p: return
        d = pathlib.Path(p)
        if not d.is_dir(): return
        sp = str(d.resolve())
        if sp in seen: return
        try:
            h = os.add_dll_directory(sp)  # keep handle alive
        except Exception:
            return
        _DLL_DIR_HANDLES.append(h); seen.add(sp); added.append(sp)

    add_dir(os.environ.get("IS_SDK_DLL_DIR"))                  # env override
    br = SDK_DIR / "build-release"
    if br.is_dir():
        for dll in br.rglob("*.dll"):                          # only real DLL dirs
            add_dir(dll.parent)

    _DLLS_ADDED = True
    if verbose:
        print(f"[dllpath] added {len(added)} dirs")
        for sp in added: print("   ", sp)

def _probe_pyd_load() -> None:
    try:
        pkg = importlib.import_module("inertialsense.logs")
        pyd_dir = pathlib.Path(pkg.__file__).parent
        # Find the compiled extension (platform tag varies)
        candidates = list(pyd_dir.glob("log_reader*.pyd"))
        if not candidates:
            print("[probe] no log_reader*.pyd found in", pyd_dir); return
        ctypes.CDLL(str(candidates[0]))
        print("[probe] log_reader loaded OK via ctypes")
    except OSError as e:
        print("[probe] log_reader failed:", e)
    except Exception as e:
        print("[probe] unexpected error:", e)

# ---------- Helpers ----------
def run_setup_command(command, cwd: os.PathLike | None = None) -> int:
    """Run 'python setup.py <args>' using the current interpreter."""
    args = command.split() if isinstance(command, str) else list(command)
    cmd = [PY, "setup.py", *args]
    print("CMD:", " ".join(cmd), f"(cwd={cwd or os.getcwd()})")
    try:
        result = subprocess.run(
            cmd,
            cwd=cwd,
            check=True,
            text=True,
            capture_output=True,
        )
        if result.stdout:
            print(result.stdout)
        return 0
    except subprocess.CalledProcessError as e:
        print("Error while running command:")
        if e.stdout:
            print("---- stdout ----")
            print(e.stdout)
        if e.stderr:
            print("---- stderr ----")
            print(e.stderr)
        return e.returncode

def run_clean(python_dir: os.PathLike = PYTHON_DIR) -> int:
    """Clean build artifacts under python_dir."""
    if not python_dir:  # default to PYTHON_DIR if python_dir is None or falsy
        python_dir = PYTHON_DIR
    print("=== Running make clean... ===")
    rc = run_setup_command(["clean"], cwd=python_dir)
    if rc:
        return rc

    pdir = pathlib.Path(python_dir)
    patterns = [
        "tmp", "build", "dist", "*.egg-info",
        "log_reader.cpython*",
        "*.so", "*.pyd", "*.pyc",
    ]
    try:
        for pat in patterns:
            for p in pdir.glob(pat):
                if p.is_dir():
                    shutil.rmtree(p, ignore_errors=True)
                elif p.is_file():
                    try:
                        p.unlink()
                    except FileNotFoundError:
                        pass
        for root, dirs, _ in os.walk(pdir):
            for d in list(dirs):
                if d == "__pycache__":  # remove caches
                    shutil.rmtree(os.path.join(root, d), ignore_errors=True)
        return 0
    except Exception as e:
        print(f"Error during clean operation: {e}")
        return 1

# ---------- Build ----------
def run_build(args: list[str] = []) -> int:
    build_type = "Release"
    clean = False

    for arg in args:
        if arg in ("-c", "--clean"):
            clean = True
        elif arg in ("-d", "--debug"):
            build_type = "Debug"

    # Use current interpreter for pip
    pip_install_cmd = [PY, "-m", "pip", "install", str(PYTHON_DIR)]
    if not in_venv() and sys.version_info >= (3, 11):
        pip_install_cmd.append("--break-system-packages")

    if clean:
        return run_clean(PYTHON_DIR)

    print("Building IS-SDK")
    result = BuildTestManager.static_build_cmake("IS_SDK_lib", SDK_DIR, is_windows=IS_WINDOWS)
    if result:
        return result

    print(f"=== Running make... ({build_type}) ===")

    print("CMD:", " ".join(pip_install_cmd))
    build_process = subprocess.run(pip_install_cmd, cwd=SDK_DIR, check=True)

    # Build extension in-place
    return run_setup_command("build_ext --inplace", cwd=PYTHON_DIR)

@contextmanager
def _argv(temp: list[str]):
    saved = sys.argv[:]
    try:
        sys.argv = temp
        yield
    finally:
        sys.argv = saved

def launch_log_inspector(open_dir: str | None = None, internal: bool | None = None) -> None:
    _add_sdk_dll_dirs()
    # Proactively load native extension so its dependencies resolve with the added DLL dirs
    try:
        _probe_pyd_load()     # safe; only prints on failure
    except Exception:
        pass
    args = ["logInspector"] + ([open_dir] if open_dir else [])
    with _argv(args):
        runpy.run_module(("inertialsense.logInspector.logInspectorInternal" if "--run-internal" in sys.argv else "inertialsense.logInspector.logInspector"), run_name="__main__")

# ---------- CLI ----------
def main() -> int:
    import argparse
    parser = argparse.ArgumentParser(description="Build or run Log Inspector")
    parser.add_argument("-c", "--clean", action="store_true", help="Clean Python build artifacts and exit")
    parser.add_argument("-d", "--debug", action="store_true", help="Build Debug (currently informational)")
    parser.add_argument("-r", "--run", action="store_true", help="Launch Log Inspector after a successful build")
    parser.add_argument("-ri", "--run-internal", action="store_true", help="Launch Log Inspector in internal mode after a successful build")
    parser.add_argument("-n", "--no-build", action="store_true", help="Skip build step and just run Log Inspector")
    parser.add_argument("--open", help="Log directory to auto-open in Log Inspector")
    args = parser.parse_args()

    if not in_venv():
        print("WARNING: Not running inside a virtual environment. sys.executable =", sys.executable)

    # Clean only
    if args.clean:
        return run_clean(PYTHON_DIR)

    # Build
    if not args.no_build:
        rc = run_build(["--debug"] if args.debug else [])
        if rc:
            return rc

    # Run only if requested
    if args.run or args.run_internal:
        launch_log_inspector(open_dir=args.open, internal=args.run_internal)

    return 0

if __name__ == "__main__":
    raise SystemExit(main())
