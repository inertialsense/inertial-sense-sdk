import os
import platform
import shutil
import sys
import subprocess
import time
from pathlib import Path
sdk_scripts_dir = Path(__file__).resolve().parent
imx_scripts_dir = sdk_scripts_dir.parent.parent
sys.path.append(sdk_scripts_dir)

class BuildTestManager:
    def __init__(self):
        self.is_windows = os.name == 'nt' or platform.system() == 'Windows'
        self.generate_release = False
        self.release_name = None
        self.release_dir = None
        self.project_name = None
        self.project_dir = None
        self.build_success = []
        self.build_failure = []
        self.test_name = None
        self.exec_name = None
        self.test_success = []
        self.test_failure = []
        self.build_type = "Release"
        self.run_build = True
        self.run_clean = False
        self.run_test = False
        self.result = 0

        # Parse command-line arguments
        self.args = sys.argv[1:]
        no_dash_args = []
        for arg in self.args:
            if   arg in ("-b", "--build"):
                self.run_build = True
            elif arg in ("-c", "--clean"):
                self.run_clean = True
                self.run_test = False
            elif arg in ("-d", "--debug"):
                self.build_type = "Debug"
            elif arg in ("-h", "--help"):
                self.print_help_menu()
                sys.exit(0)
            elif arg in ("-n", "--nobuild"):
                self.run_build = False
            elif arg in ("-t", "--test", "--tests"):
                self.run_test = True
            elif arg in ("-g", "--generate-release"):
                self.generate_release = True
            else:
                no_dash_args.append(arg)

        # Parse project name and directory
        if 0 < len(no_dash_args):
            self.project_name = no_dash_args[0]
        if 1 < len(no_dash_args):
            self.project_dir = os.path.abspath(no_dash_args[1])
            if not os.path.exists(self.project_dir):
                self.project_dir = sdk_scripts_dir / no_dash_args[1]
                if not os.path.exists(self.project_dir):
                    self.project_dir = imx_scripts_dir / no_dash_args[1]
                    if not os.path.exists(self.project_dir):
                        raise TypeError(f"Project directory not found: {type(self.project_dir).__name__}.")
        if 2 < len(no_dash_args):
            self.exec_name = no_dash_args[2]

    def set_release_info(self, release_name=None, release_dir=None):
        self.release_name = release_name
        self.release_dir = release_dir
        self.print_release_info()

    def print_help_menu(self):
        print("Help Menu:")
        print(" -b --build              Enable build (default on)")
        print(" -c --clean              Clean build")
        print(" -d --debug              Enable debug build")
        print(" -h --help               Show this menu")
        print(" -n --nobuild            Disable build")
        print(" -t --test               Run tests")
        print(" -g --generate-release   Generate software release")

    def print_release_info(self):
        if self.generate_release:
            self.print_ylw(f"==========================================")
            self.print_ylw(f" Generating: RELEASE {self.release_name}")
            self.print_ylw(f" Directory:  {self.release_dir} ")
            self.print_ylw(f"==========================================")
            print("")
            time.sleep(1)

    def print_red(self, str): print(f"\033[31m{str}\033[0m")            # ANSI Red
    def print_grn(self, str): print(f"\033[32m{str}\033[0m")            # ANSI Green
    def print_blu(self, str): print(f"\033[34m{str}\033[0m")            # ANSI Blue
    def print_cyn(self, str): print(f"\033[36m{str}\033[0m")            # ANSI Cyan
    def print_ylw(self, str): print(f"\033[33m{str}\033[0m")            # ANSI Yellow

    def print_bright_red(self, str): print(f"\033[1;31m{str}\033[0m")   # ANSI Bold Red
    def print_bright_grn(self, str): print(f"\033[1;32m{str}\033[0m")   # ANSI Bold Green
    def print_bright_blu(self, str): print(f"\033[1;34m{str}\033[0m")   # ANSI Bold Blue
    def print_bright_cyn(self, str): print(f"\033[1;36m{str}\033[0m")   # ANSI Bold Cyan
    def print_bright_ylw(self, str): print(f"\033[1;33m{str}\033[0m")   # ANSI Bold Yellow

    def build_header(self, name):
        self.project_name = name
        if self.run_clean:
            action = "CLEAN"
        else:
            action = "BUILD"

        self.print_blu(f"==========================================")
        self.print_blu(f" {action}:  {self.project_name}")
        self.print_blu(f"==========================================")

    def build_footer(self, exit_code):  
        if exit_code:
            self.print_red(f"[***** BUILD: {self.project_name} - FAILED *****]")
            self.build_failure.append(self.project_name)
        else:
            self.print_grn(f"[BUILD: {self.project_name} - Passed]")
            self.build_success.append(self.project_name)
        print("")

    def test_header(self, name):
        self.test_name = name
        self.print_cyn(f"==========================================")
        self.print_cyn(f" TEST:  {self.test_name}")
        self.print_cyn(f"==========================================")

    def test_footer(self, exit_code):  
        if exit_code:
            self.print_red(f"[***** TEST: {self.test_name} - FAILED *****]")
            self.test_failure.append(self.test_name)
        else:
            self.print_grn(f"[TEST: {self.test_name} - Passed]")
            self.test_success.append(self.test_name)
        print("")

    def print_summary(self):
        if self.run_build:
            if self.run_clean:
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
            print("")
        if self.run_test:
            self.print_cyn(f"==========================================")
            self.print_cyn(f" TEST SUMMARY:")
            self.print_cyn(f"==========================================")
            if self.test_success:
                self.print_grn(f"[PASSED]: " + ", ".join(self.test_success))
            if self.test_failure:
                self.print_red(f"[FAILED]: " + ", ".join(self.test_failure))
            print("")
        self.print_release_info()

    def build_callback(self, project_name, callback):
        if not self.run_build:
            return
        result = 0
        self.build_header(project_name)
        result = callback(self.args)
        self.build_footer(result)
        if result:
            self.result = result
        return result
    
    def test_callback(self, project_name, callback):
        result = 0
        if self.run_test:
            self.test_header(project_name)
            result = callback()
            self.test_footer(result)
        if result:
            self.result = result
        return result

    def build_script(self, project_name, script_path, args=[]):
        if not self.run_build:
            return
        if self.is_windows:
            command = ["cmd", "/c", str(script_path)]
        else:
            command = ["bash", str(script_path)]
        if args:
            command.extend(args)
        if self.args:
            command.extend(self.args)
        self.build_header(project_name)
        result = 0
        try:
            # Redirect input from os.devnull to suppress 'pause' commands
            with open(os.devnull, 'r') as devnull:
                subprocess.run(command, check=True, stdin=devnull, cwd=os.path.dirname(script_path))            
        except subprocess.CalledProcessError as e:
            print(f"Error building {project_name}!")
            result = e.returncode
            
        self.build_footer(result)
        if result:
            self.result = result
        return result

    def build_cmake(self, project_name, project_dir):
        if not self.run_build:
            return
        project_dir = Path(project_dir)

        self.build_header(project_name)
        result = self.static_build_cmake(project_name, project_dir, self.build_type, self.run_clean)
        self.build_footer(result)
        if result:
            self.result = result
        return result

    @staticmethod
    def is_directory_empty(directory):
        if os.path.exists(directory) and os.path.isdir(directory):
            return len(os.listdir(directory)) == 0    
    
    @staticmethod
    def static_build_cmake(project_name, project_dir, build_type="Release", clean=False):
        result = 0
        if clean:
            build_dir = project_dir / "build"
            print(f"=== Running make clean... ===")
            try:
                if os.path.exists(build_dir):
                    shutil.rmtree(build_dir)
            except Exception as e:
                print(f"Error cleaning: {project_name}.  Directory `{build_dir}` cannot be removed.")
                if not BuildTestManager.is_directory_empty(build_dir):
                    result = -1
        else:   # Build process
            print(f"=== Running make... ({build_type}) ===")
            try:
                num_proc = os.cpu_count()
                num_proc = int(max(num_proc - num_proc/10, 6))
                subprocess.check_call(["cmake", "-B", "build", "-S", ".", f"-DCMAKE_BUILD_TYPE={build_type}"], cwd=str(project_dir))
                subprocess.check_call(["cmake", "--build", "build", "--config", f"{build_type}", "-j", f"{num_proc}"], cwd=str(project_dir))
            except subprocess.CalledProcessError as e:
                print(f"Error building {project_name}!")
                result = e.returncode
        return result

    def test_exec(self, test_name, test_dir, exec_name=""):
        if not self.run_test:
            return
        test_dir = str(test_dir) + "/build"
        if not exec_name:
            exec_name = test_name
        if self.is_windows:
            test_dir = test_dir + "/Release"
            exec_name = exec_name + ".exe"
        else:
            exec_name = "./" + exec_name

        # Normalize paths
        test_dir = os.path.normpath(test_dir)
        exec_path = os.path.join(test_dir, exec_name)
        
        # Debug prints
        print(f"test_dir: {test_dir}")
        print(f"exec_name: {exec_name}")
        print(f"exec_path: {exec_path}")
        
        # Check if paths exist
        if not os.path.isdir(test_dir):
            raise FileNotFoundError(f"Directory not found: {test_dir}")
        if not os.path.isfile(exec_path):
            raise FileNotFoundError(f"Executable not found: {exec_path}")

        self.test_header(test_name)
        result = 0
        try:
            subprocess.check_call(exec_path, cwd=test_dir)
        except subprocess.CalledProcessError as e:
            print(f"Error testing {test_name}!")
            result = e.returncode
        self.test_footer(result)
        if result:
            self.result = result
        return result
    
    def clean_rm(self, path):
        if self.run_clean:  # Only clean if clean option was specified
            if os.path.exists(path):
                if os.path.isdir(path):
                    shutil.rmtree(path)  # Remove the directory and its contents
                    print(f"Directory removed: {path}")
                elif os.path.isfile(path):
                    os.remove(path)  # Remove the file
                    print(f"File removed: {path}")
                else:
                    raise FileNotFoundError(f"Path does not exist: {path}")

    @staticmethod
    def source_bash_script(script_path):
        """
        Sources a bash script and returns the resulting environment variables.

        :param script_path: Path to the bash script.
        :return: A dictionary of the environment variables after sourcing the script.
        """
        # Command to source the script and print the environment
        command = f"bash -c 'source {script_path} && env'"
        
        # Run the command and capture the output
        result = subprocess.run(command, shell=True, text=True, capture_output=True, check=True)
        
        # Parse the environment variables from the output
        env_vars = {}
        for line in result.stdout.splitlines():
            key, _, value = line.partition("=")
            env_vars[key] = value

        return env_vars

    def run(self):

        # Execute build and/or test based on flags
        result = 0
        if self.run_build:
            result = self.build_cmake(self.project_name, self.project_dir)
            if result:
                sys.exit(result)

        if self.run_test:
            result = self.test_exec(self.project_name, self.project_dir, self.exec_name)
            if result:
                sys.exit(result)

        sys.exit(0)

if __name__ == "__main__":
    bm = BuildTestManager()
    bm.run()
