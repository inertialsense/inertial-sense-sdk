import glob
import os
import platform
import sys
import subprocess
import shutil
from pathlib import Path
from build_manager import BuildTestManager


sdk_dir = Path(__file__).resolve().parent.parent
log_inspector_dir = sdk_dir / "python"/ "logInspector/"
is_windows = os.name == 'nt' or platform.system() == 'Windows'
if is_windows:
    python_exec = "python"
else:   # Linux
    python_exec = "python3"

def run_setup_command(command):
    try:
        # Run the setup.py command
        command = [python_exec, "setup.py"] + command.split()
        result = subprocess.run(
            command,
            check=True,
            text=True,
            capture_output=True
        )
        if result.returncode:
            print(f"Setup.py failure: {command}")
        return result.returncode
    except subprocess.CalledProcessError as e:
        print("Error while running command:")
        print(e.stderr)
        return e.returncode

def run_clean():
    os.chdir(log_inspector_dir)

    print("=== Running make clean... ===")
    result = run_setup_command("clean")
    if result:
        return result

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

    pip_install_command = f"pip3 install {log_inspector_dir}"
    version_info = sys.version_info
    if version_info.major > 3 or (version_info.major == 3 and version_info.minor >= 11):
        pip_install_command += " --break-system-packages"

    if clean:
        return run_clean()
    else:
        print("Building IS-SDK")
        is_windows = os.name == 'nt' or platform.system() == 'Windows'
        result = BuildTestManager.static_build_cmake("IS_SDK_lib", sdk_dir, is_windows=is_windows)
        if result: 
            sys.exit(result)

        print(f"=== Running make... ({build_type}) ===")
        print(pip_install_command)
        build_process = subprocess.run(pip_install_command, shell=True)
        if build_process.returncode:
            print("pip install failed!")
            return build_process.returncode

        os.chdir(log_inspector_dir)
        return run_setup_command("build_ext --inplace")
        # return run_setup_command("bdist_wheel")

def main():
    clean = False
    debug = False
    build_type = "Release"
    args = sys.argv[1:]

    if clean:
        run_clean()
    else:
        run_build(args)

if __name__ == "__main__":
    main()
