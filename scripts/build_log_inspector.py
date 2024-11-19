import glob
import os
import platform
import sys
import subprocess
import shutil
from pathlib import Path

sdk_dir = Path(__file__).resolve().parent.parent
log_inspector_dir = sdk_dir / "python"/ "logInspector/"
is_windows = os.name == 'nt' or platform.system() == 'Windows'


def source_virtualenv():
    global sdk_dir
    virtualenv_path = sdk_dir / "script" / "lib" / "python_venv.sh"
    if os.path.exists(virtualenv_path):
        subprocess.run(f"source {virtualenv_path}", shell=True, executable="/bin/bash")
    else:
        raise FileNotFoundError(f"Virtual environment script not found: {virtualenv_path}")

def run_clean():
    global log_inspector_dir
    os.chdir(log_inspector_dir)

    print("=== Running make clean... ===")
    paths_to_remove = [
        "tmp", "build", "log_reader.egg-info", "log_reader.cpython*",
        "*.so", "*.pyc"
    ]
    try:
        for path in paths_to_remove:
            for matched_path in glob.glob(path):
                if os.path.isdir(matched_path):
                    shutil.rmtree(matched_path)
                elif os.path.isfile(matched_path):
                    os.remove(matched_path)
        
        for root, dirs, _ in os.walk(".."):
            for directory in dirs:
                if directory == "__pycache__":
                    shutil.rmtree(os.path.join(root, directory))
        return 0
    
    except Exception as e:
        print(f"Error during clean operation: {e}")
        return 1
    
def run_build(args=[]):
    build_type="Release"
    clean = False

    # Parse command-line arguments
    for arg in args:
        if arg in ("-c", "--clean"):
            clean = True
        elif arg in ("-d", "--debug"):
            build_type = "Debug"

    global log_inspector_dir
    pip_install_command = f"pip3 install {log_inspector_dir}"
    version_info = sys.version_info
    if version_info.major > 3 or (version_info.major == 3 and version_info.minor >= 11):
        pip_install_command += " --break-system-packages"

    if clean:
        return run_clean()
    else:
        print(f"=== Running make... ({build_type}) ===")

        os.chdir(log_inspector_dir.parent)
        print(pip_install_command)
        build_process = subprocess.run(pip_install_command, shell=True)
        if build_process.returncode:
            print("pip install failed!")
            return build_process.returncode

        os.chdir(log_inspector_dir)
        if is_windows:
            cmd = "python"
        else:
            cmd = "python3"
        build_process = subprocess.run([cmd, "setup.py", "build_ext", "--inplace"])
        if build_process.returncode:
            print(f"{cmd} setup build failed!")
            return build_process.returncode
        return 0

def main():
    clean = False
    debug = False
    build_type = "Release"

    for arg in sys.argv[1:]:
        if arg in ("-c", "--clean"):
            clean = True
        elif arg in ("-d", "--debug"):
            debug = True
            build_type = "Debug"

    if clean:
        run_clean()
    else:
        run_build(build_type)

if __name__ == "__main__":
    main()
