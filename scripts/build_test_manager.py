import os
import shutil
import sys
import subprocess
from colorama import Fore, Style, init
from pathlib import Path

class BuildTestManager:
    def __init__(self):
        self.script_dir = Path(__file__).resolve().parent

        self.build_name = ""
        self.build_success = []
        self.build_failure = []
        self.last_build_success=0

        # Initialize colorama
        init(autoreset=True)

    def print_red(self, str): print(Fore.RED + str)
    def print_grn(self, str): print(Fore.GREEN + str)
    def print_blu(self, str): print(Fore.BLUE + str)
    # print(Style.BRIGHT + "This is bright text")
    # print(Style.RESET_ALL + "Back to normal text")

    def build_header(self, name):
        self.build_name = name
        self.print_blu(f"==========================================")
        self.print_blu(f" BUILD:  {self.build_name}")
        self.print_blu(f"==========================================")

    def build_footer(self, exit_code):  
        if exit_code:
            self.print_red(f"[***** BUILD: {self.build_name} - FAILED *****]")
            self.build_failure.append(self.build_name)
            last_build_success=0
        else:
            self.print_grn(f"[BUILD: {self.build_name} - Passed]")
            self.build_success.append(self.build_name)
            last_build_success=1
        print("")

    def build_summary(self):
        self.print_blu(f"==========================================")
        self.print_blu(f" BUILD SUMMARY:")
        self.print_blu(f"==========================================")
        if self.build_success:
            self.print_grn(f"[PASSED]: " + ", ".join(self.build_success))
        if self.build_failure:
            self.print_red(f"[FAILED]: " + ", ".join(self.build_failure))
        print("")

        # if [ -n "$RELEASE_BUILD" ]; then 
        #     echo_yellow_title "Generated: RELEASE ${RELEASE_NAME}"
        # fi


# Import external scripts (you would need to convert these bash scripts to Python modules or functions)
# from lib.echo_color import build_header, build_footer, tests_header, tests_footer
# from lib.results_build import build_header, build_footer
# from lib.results_tests import tests_header, tests_footer

    def build_cmake(self, project_name, project_dir, args=[]):
        project_dir = Path(project_dir)
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

        build_dir = project_dir / "build"
        exit_code = 0

        if clean:
            print(f"\n=== Running make clean... ===")
            try:
                shutil.rmtree(build_dir)
            except OSError as e:
                print(f"An error occurred while trying to delete {build_dir}: {e}")
                exit_code = e.errno

        else:   # Build process
            self.build_header(project_name)
            print(f"\n=== Running make... ({build_type}) ===")
            try:
                # build_dir.mkdir(exist_ok=True)
                subprocess.check_call(["cmake", "-B", "build", "-S", ".", "-DCMAKE_BUILD_TYPE={build_type}"], cwd=str(project_dir))
                subprocess.check_call(["cmake", "--build", "./build", "--config", "{build_type}", "-j", f"{os.cpu_count()-4}"], cwd=str(project_dir))
            except subprocess.CalledProcessError as e:
                print(f"Error building {project_name}!")
                exit_code = e.returncode
            self.build_footer(exit_code)

        return exit_code

    def test_cmake(self, *args):
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

        # os.chdir(script_dir)
        return test_result == 0

    def build_test_cmake(self, *args):
        return self.build_cmake(*args) and self.test_cmake(*args)

    def run(self, args):

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
            success = self.build_cmake(*args)
            if not success:
                sys.exit(1)

        if test and success:
            success = self.test_cmake(*args)
            if not success:
                sys.exit(1)

        sys.exit(0 if success else 1)

if __name__ == "__main__":
    bm = BuildTestManager()
    args = sys.argv[1:]
    bm.run(args)