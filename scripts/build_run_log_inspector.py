import os
import subprocess
import sys
import build_log_inspector

def source_virtualenv():
    script_dir = os.path.dirname(os.path.realpath(__file__))
    virtualenv_path = os.path.join(script_dir, "lib", "python_venv.sh")
    if os.path.exists(virtualenv_path):
        subprocess.run(f"source {virtualenv_path}", shell=True, executable="/bin/bash")
    else:
        raise FileNotFoundError(f"Virtual environment script not found: {virtualenv_path}")

def source_script(script_name):
    script_dir = os.path.dirname(os.path.realpath(__file__))
    script_path = os.path.join(script_dir, script_name)
    if os.path.exists(script_path):
        subprocess.run(f"source {script_path}", shell=True, executable="/bin/bash")
    else:
        raise FileNotFoundError(f"Script not found: {script_path}")

def build_header(module_name):
    print(f"Building {module_name}...")

def build_footer(exit_code):
    if exit_code == 0:
        print("Build completed successfully.")
    else:
        print("Build failed.")
    return exit_code

def main():
    script_dir = os.path.dirname(os.path.realpath(__file__))
    os.chdir(script_dir)

    # Source necessary scripts
    source_virtualenv()
    source_script("./lib/echo_color.sh")
    source_script("./lib/results_build.sh")
    source_script("./lib/results_tests.sh")

    # Build header
    build_header("logInspector")

    # Build Log Inspector
    build_log_inspector.run_build()

    # Run Log Inspector
    log_inspector_dir = os.path.join(script_dir, "../python/logInspector")
    os.chdir(log_inspector_dir)
    python_process = subprocess.run(["python3", "logInspectorInternal.py"], shell=True)
    os.chdir(script_dir)

    # Combine build and test results
    tests_exit_code = python_process.returncode
    total_exit_code = build_exit_code + tests_exit_code

    # Print results
    print(f"BUILD EXIT CODE: {build_exit_code}")
    print(f"TESTS EXIT CODE: {tests_exit_code}")
    print(f"TOTAL EXIT CODE: {total_exit_code}")

    # Return results
    sys.exit(total_exit_code)

if __name__ == "__main__":
    main()
