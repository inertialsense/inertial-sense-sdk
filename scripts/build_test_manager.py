import os
import shutil
import sys
import subprocess
from colorama import Fore, Style, init
from pathlib import Path
sdk_scripts_dir = Path(__file__).resolve().parent
sys.path.append(sdk_scripts_dir)
from lib import python_venv

class BuildTestManager:
    def __init__(self):
        python_venv.activate_virtualenv()

        self.release_build = None
        self.build_name = ""
        self.build_success = []
        self.build_failure = []
        self.last_build_success=0
        self.build_type = "Release"
        self.build = False
        self.clean = False
        self.test = False

        # Parse command-line arguments
        self.args = sys.argv[1:]
        for arg in self.args:
            if arg in ("-b", "--build"):
                self.build = True
            elif arg in ("-c", "--clean"):
                self.clean = True
            elif arg in ("-d", "--debug"):
                self.build_type = "Debug"
            elif arg in ("-t", "--test"):
                self.test = True

        # Initialize colorama
        init(autoreset=True)

    def print_red(self, str): print(Fore.RED + str)
    def print_grn(self, str): print(Fore.GREEN + str)
    def print_blu(self, str): print(Fore.BLUE + str)
    def print_ylw(self, str): print(Fore.YELLOW + str)
    # print(Style.BRIGHT + "This is bright text")
    # print(Style.RESET_ALL + "Back to normal text")

    def header(self, name):
        self.build_name = name
        if self.clean:
            action = "CLEAN"
        else:
            action = "BUILD"

        self.print_blu(f"==========================================")
        self.print_blu(f" {action}:  {self.build_name}")
        self.print_blu(f"==========================================")

    def footer(self, exit_code):  
        if exit_code:
            self.print_red(f"[***** BUILD: {self.build_name} - FAILED *****]")
            self.build_failure.append(self.build_name)
            self.last_build_success=0
        else:
            self.print_grn(f"[BUILD: {self.build_name} - Passed]")
            self.build_success.append(self.build_name)
            self.last_build_success=1
        print("")

    def print_build_summary(self):
        if self.clean:
            action = "CLEAN"
        else:
            action = "BUILD"

        self.print_blu(f"==========================================")
        self.print_blu(f" {action} SUMMARY:")
        self.print_blu(f"==========================================")
        if self.build_success:
            self.print_grn(f"[PASSED]: " + ", ".join(self.build_success))
        if self.build_failure:
            self.print_red(f"[FAILED]: " + ", ".join(self.build_failure))
        if self.release_build:
            self.print_ylw(f"Generated: RELEASE {self.release_build}")
        print("")

        # if [ -n "$RELEASE_BUILD" ]; then 
        #     echo_yellow_title "Generated: RELEASE ${RELEASE_NAME}"
        # fi

    def build_callback(self, project_name, func_ptr):
        self.header(project_name)
        result = func_ptr(self.args)
        self.footer(result)
        return result

    def build_script(self, project_name, script_path, args):
        command = ["bash", str(script_path)]
        if args:
            command.extend(args)
        if self.args:
            command.extend(self.args)
        self.header(project_name)

        result = 0
        try:
            subprocess.run(command, check=True)
        except subprocess.CalledProcessError as e:
            print(f"Error building {project_name}!")
            result = e.returncode
            
        self.footer(result)
        return result

    def build_cmake(self, project_name, project_dir):
        project_dir = Path(project_dir)
        build_dir = project_dir / "build"
        result = 0

        self.header(project_name)
        if self.clean:
            if os.path.exists(build_dir):
                shutil.rmtree(build_dir)

        else:   # Build process
            print(f"=== Running make... ({self.build_type}) ===")
            try:
                subprocess.check_call(["cmake", "-B", "build", "-S", ".", "-DCMAKE_BUILD_TYPE={build_type}"], cwd=str(project_dir))
                subprocess.check_call(["cmake", "--build", "./build", "--config", "{build_type}", "-j", f"{os.cpu_count()-4}"], cwd=str(project_dir))
            except subprocess.CalledProcessError as e:
                print(f"Error building {project_name}!")
                result = e.returncode
        self.footer(result)
        return result

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

        return test_result == 0

    def build_test_cmake(self, *args):
        return self.build_cmake(*args) and self.test_cmake(*args)
    
    def clean_rm(self, path):
        if self.clean:  # Only clean if clean option was specified
            if os.path.exists(path):
                if os.path.isdir(path):
                    shutil.rmtree(path)  # Remove the directory and its contents
                    print(f"Directory removed: {path}")
                elif os.path.isfile(path):
                    os.remove(path)  # Remove the file
                    print(f"File removed: {path}")
                else:
                    raise FileNotFoundError(f"Path does not exist: {path}")

    def run(self):

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
    bm.run()