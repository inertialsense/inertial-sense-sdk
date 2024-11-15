import os
import shutil
import sys
import subprocess
from pathlib import Path

script_dir = Path(__file__).resolve().parent

# Import external scripts (you would need to convert these bash scripts to Python modules or functions)
# from lib.echo_color import build_header, build_footer, tests_header, tests_footer
# from lib.results_build import build_header, build_footer
# from lib.results_tests import tests_header, tests_footer

def build_cmake(*args):
    project_name = ""
    project_dir = ""
    clean = False
    build_type = "Release"
    parsed_args = []

    # Parse arguments
    # args = list(args)
    for arg in args:
        if arg in ("-c", "--clean"):
            clean = True
        elif arg in ("-d", "--debug"):
            build_type = "Debug"
        elif not arg.startswith("-"):
            parsed_args.append(arg)

    if len(parsed_args) >= 1:
        project_name = parsed_args[0]
    if len(parsed_args) >= 2:
        project_dir = Path(parsed_args[1])

    build_dir = project_dir / "build"
    exit_code = 0

    if clean:
        print("\n=== Running make clean... ===")
        try:
            shutil.rmtree(build_dir)
        except OSError as e:
            print(f"An error occurred while trying to delete {build_dir}: {e}")
            exit_code = e.errno

    else:   # Build process
        # build_header(project_name)  # Assuming this function is defined in echo_color.sh or similar
        print(f"\n\n=== Running make... ({build_type}) ===")
        try:
            # build_dir.mkdir(exist_ok=True)
            subprocess.check_call(["cmake", "-B", "build", "-S", ".", "-DCMAKE_BUILD_TYPE={build_type}"], cwd=str(project_dir))
            subprocess.check_call(["cmake", "--build", "./build", "--config", "{build_type}", "-j", f"{os.cpu_count()-4}"], cwd=str(project_dir))
        except subprocess.CalledProcessError as e:
            print("Error building {project_name}!")
            exit_code = e.returncode
        # build_footer(build_result)  # Assuming this function is defined in results_build.sh or similar

    return exit_code

def test_cmake(*args):
    parsed_args = []
    args = list(args)

    # Parse arguments
    while args:
        arg = args.pop(0)
        if not arg.startswith("-"):
            parsed_args.append(arg)

    testname = parsed_args[0] if len(parsed_args) >= 1 else ""
    cmakelists_dir = parsed_args[1] if len(parsed_args) >= 2 else ""
    execname = parsed_args[2] if len(parsed_args) >= 3 else "run_tests"

    cmakelists_path = Path(cmakelists_dir)
    os.chdir(cmakelists_path / "build")

    # tests_header(testname)  # Assuming this function is defined in echo_color.sh or similar
    result = subprocess.run([f"./{execname}", "--gtest_color=yes"], check=False)
    test_result = result.returncode
    # tests_footer(test_result)  # Assuming this function is defined in results_tests.sh or similar

    os.chdir(script_dir)
    return test_result == 0

def build_test_cmake(*args):
    return build_cmake(*args) and test_cmake(*args)

def run(*args):

    # Default configuration
    build_type = "Release"
    build = False
    clean = False
    test = False

    # Parse command-line arguments
    for arg in args:
        if arg in ("-b", "--build"):
            build = True
        elif arg in ("-c", "--clean"):
            clean = True
        elif arg in ("-d", "--debug"):
            build_type = "Debug"
        elif arg in ("-t", "--test"):
            test = True

    # Execute build and/or test based on flags
    success = True
    if build:
        success = build_cmake(*args)
        if not success:
            sys.exit(1)

    if test and success:
        success = test_cmake(*args)
        if not success:
            sys.exit(1)

    sys.exit(0 if success else 1)

if __name__ == "__main__":
    args = sys.argv[1:]
    run(args)