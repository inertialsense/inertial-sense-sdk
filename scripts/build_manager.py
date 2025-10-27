import argparse
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


def parse_args(argv: list[str] | None = None):
    """
    Parse CLI using argparse while preserving backward-compatible flags.

    Positional (all optional, same order as before):
      project_name [project_dir] [exec_name]

    Key flags (compatible with prior script):
      -b/--build (default on), -n/--no-build
      -c/--clean
      -t/--test/--tests
      -d/--debug  (sets build type Debug)
      -g/--generate-release
      --release-name NAME
      --release-dir DIR
      --config {Release,Debug}  (overrides -d if provided)
    """
    parser = argparse.ArgumentParser(
        prog="build_manager.py",
        description="Build/Test manager for CMake projects with optional release packaging."
    )

    # Positional (all optional)
    parser.add_argument("project_name", nargs="?", help="Project name")
    parser.add_argument("project_dir", nargs="?", help="Project directory")
    parser.add_argument("exec_name",   nargs="?", help="Executable name (without extension)")

    # Build enable/disable (default = build)
    g_build = parser.add_mutually_exclusive_group()
    g_build.add_argument("-b", "--build", dest="build", action="store_true",
                         help="Enable build (default)")
    g_build.add_argument("-n", "--no-build", dest="build", action="store_false",
                         help="Disable build")
    parser.set_defaults(build=True)

    parser.add_argument("-c", "--clean", action="store_true", help="Clean build")
    parser.add_argument("-t", "--test", "--tests", dest="test", action="store_true",
                        help="Run tests")

    # Build type: -d maps to Debug; --config explicitly sets
    parser.add_argument("-d", "--debug", dest="debug", action="store_true",
                        help="Debug build (same as --config Debug)")
    parser.add_argument("--config", choices=["Release", "Debug"],
                        help="Explicit CMAKE_BUILD_TYPE (overrides -d if set)")

    # Release generation
    parser.add_argument("-g", "--generate-release", dest="generate_release",
                        action="store_true", help="Generate software release")
    parser.add_argument("--release-name", help="Release tag/name")
    parser.add_argument("--release-dir",  help="Output directory for the release")

    # Parse, but also keep unknown args to forward into child build scripts
    args, unknown = parser.parse_known_args(argv)
    args.forward_args = sys.argv[1:]
    return args


class BuildTestManager:
    def __init__(self, args=None):
        if args is None:
            args = parse_args(sys.argv[1:])
        self.args = args
        self.is_windows = os.name == 'nt' or platform.system() == 'Windows'
        self.generate_release = bool(args.generate_release)
        self.release_name = args.release_name
        self.release_dir = args.release_dir

        self.project_name = args.project_name
        self.project_dir = None
        self.exec_name = args.exec_name

        self.build_success = []
        self.build_failure = []
        self.test_name = None
        self.test_success = []
        self.test_failure = []

        # Build/test switches
        self.run_clean = bool(args.clean)
        self.run_build = bool(args.build)
        self.run_test = bool(args.test)

        # Build type resolution: --config wins, else -d implies Debug, default Release
        if args.config:
            self.build_type = args.config
        elif args.debug:
            self.build_type = "Debug"
        else:
            self.build_type = "Release"

        # Preserve any unknown flags for downstream scripts (parity with previous self.args)
        self.forward_args = list(getattr(args, "forward_args", []))

        # Resolve project_dir similarly to the original script
        if args.project_dir:
            pd = os.path.abspath(args.project_dir)
            if not os.path.exists(pd):
                pd = str(sdk_scripts_dir / args.project_dir)
            if not os.path.exists(pd):
                pd = str(imx_scripts_dir / args.project_dir)
            if not os.path.exists(pd):
                raise FileNotFoundError(f"Project directory not found: {args.project_dir}")
            self.project_dir = pd

        self.result = 0

    def set_release_info(self, release_name=None, release_dir=None):
        if release_name is not None:
            self.release_name = release_name
        if release_dir is not None:
            self.release_dir = release_dir
        self.print_release_info()

    # ---- printing helpers (unchanged) ----
    def print_help_menu(self):
        print("Use --help for argparse-driven help.")

    def print_red(self, s): print(f"\033[31m{s}\033[0m")
    def print_grn(self, s): print(f"\033[32m{s}\033[0m")
    def print_blu(self, s): print(f"\033[34m{s}\033[0m")
    def print_cyn(self, s): print(f"\033[36m{s}\033[0m")
    def print_ylw(self, s): print(f"\033[33m{s}\033[0m")

    def print_bright_red(self, s): print(f"\033[1;31m{s}\033[0m")
    def print_bright_grn(self, s): print(f"\033[1;32m{s}\033[0m")
    def print_bright_blu(self, s): print(f"\033[1;34m{s}\033[0m")
    def print_bright_cyn(self, s): print(f"\033[1;36m{s}\033[0m")
    def print_bright_ylw(self, s): print(f"\033[1;33m{s}\033[0m")

    def print_release_info(self):
        if self.generate_release:
            self.print_ylw("==========================================")
            self.print_ylw(f" Generating: RELEASE {self.release_name}")
            self.print_ylw(f" Directory:  {self.release_dir} ")
            self.print_ylw("==========================================")
            print("")
            time.sleep(1)

    def build_header(self, name):
        self.project_name = name or self.project_name
        action = "CLEAN" if self.run_clean else "BUILD"
        self.print_blu("==========================================")
        self.print_blu(f" {action}:  {self.project_name}")
        self.print_blu("==========================================")

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
        self.print_cyn("==========================================")
        self.print_cyn(f" TEST:  {self.test_name}")
        self.print_cyn("==========================================")

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
            action = "CLEAN" if self.run_clean else "BUILD"
            self.print_blu("==========================================")
            self.print_blu(f" {action} SUMMARY:")
            self.print_blu("==========================================")
            if self.build_success:
                self.print_grn("[PASSED]: " + ", ".join(self.build_success))
            if self.build_failure:
                self.print_red("[FAILED]: " + ", ".join(self.build_failure))
            print("")
        if self.run_test:
            self.print_cyn("==========================================")
            self.print_cyn(" TEST SUMMARY:")
            self.print_cyn("==========================================")
            if self.test_success:
                self.print_grn("[PASSED]: " + ", ".join(self.test_success))
            if self.test_failure:
                self.print_red("[FAILED]: " + ", ".join(self.test_failure))
            print("")
        self.print_release_info()

    def build_callback(self, project_name, callback):
        if not self.run_build:
            return
        self.build_header(project_name)
        result = callback(self.forward_args)
        self.build_footer(result)
        if result:
            self.result = result
        return result

    def test_callback(self, project_name, callback):
        result = 0
        if self.run_test:
            self.test_header(project_name)
            result = callback(self.test_name)
            self.test_footer(result)
        if result:
            self.result = result
        return result

    def build_script(self, project_name, script_path, args=None):
        if not self.run_build:
            return
        args = args or []
        if self.is_windows:
            command = ["cmd", "/c", str(script_path)]
        else:
            command = ["bash", str(script_path)]
        command.extend(args)
        command.extend(self.forward_args)  # preserve pass-through
        self.build_header(project_name)
        result = 0
        try:
            with open(os.devnull, 'r') as devnull:
                build_process = subprocess.run(
                    command, check=True, stdin=devnull,
                    cwd=os.path.dirname(script_path) if os.path.dirname(script_path) else None
                )
                result = build_process.returncode
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
        result = self.static_build_cmake(project_name, project_dir, self.build_type, self.run_clean, self.is_windows)
        self.build_footer(result)
        if result:
            self.result = result
        return result

    @staticmethod
    def is_directory_empty(directory):
        return os.path.exists(directory) and os.path.isdir(directory) and len(os.listdir(directory)) == 0

    @staticmethod
    def static_build_cmake(project_name, project_dir, build_type="Release", clean=False, is_windows=False):
        result = 0
        suffix = f"-{build_type.lower()}" if is_windows else ""
        build_dir = project_dir / f"build{suffix}"
        if clean:
            print("=== Running make clean... ===")

            build_dir_o = project_dir / "out/build"
            build_dir_r = project_dir / "release"
            build_dir_d = project_dir / "debug"
            build_dir_br = project_dir / "build-release"
            build_dir_bd = project_dir / "build-debug"
            build_dir_b = project_dir / "build"

            try:
                for p in (build_dir_o, build_dir_r, build_dir_d, build_dir_br, build_dir_bd, build_dir_b):
                    if os.path.exists(p):
                        shutil.rmtree(p)
            except Exception:
                print(f"Error cleaning: {project_name}.  Directory `{build_dir}` cannot be removed.")
                if not BuildTestManager.is_directory_empty(build_dir):
                    result = -1
        else:
            print(f"=== Running make... ({build_type}) ===")
            try:
                num_proc = os.cpu_count()
                num_proc = int(max(num_proc - num_proc / 10, 6))
                start_time = time.time()

                if is_windows:
                    arch = "x64"
                    vcpkg_toolchain = r"C:\vcpkg\scripts\buildsystems\vcpkg.cmake"
                    candidate_vcvars_paths = [
                        r"C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat",
                        r"C:\Program Files\Microsoft Visual Studio\2022\VC\Auxiliary\Build\vcvarsall.bat",
                    ]
                    vcvars_path = next((p for p in candidate_vcvars_paths if os.path.exists(p)), None)
                    if not vcvars_path:
                        print("vcvarsall.bat path not found!!!")
                        return -1

                    cmd_configure = (
                        f'"{vcvars_path}" {arch} && '
                        f'cmake -G Ninja -B "{build_dir}" -S . '
                        f'-DCMAKE_BUILD_TYPE={build_type} '
                        f'-DCMAKE_TOOLCHAIN_FILE="{vcpkg_toolchain}"'
                    )
                    subprocess.check_call(cmd_configure, shell=True, cwd=str(project_dir))

                    cmd_build = f'"{vcvars_path}" {arch} && cmake --build "{build_dir}" --parallel {num_proc}'
                    subprocess.check_call(cmd_build, shell=True, cwd=str(project_dir))
                else:
                    subprocess.check_call(
                        ["cmake", "-B", "build", "-S", ".", f"-DCMAKE_BUILD_TYPE={build_type}"],
                        cwd=str(project_dir)
                    )
                    subprocess.check_call(
                        ["cmake", "--build", "build", "--config", f"{build_type}", "-j", f"{num_proc}"],
                        cwd=str(project_dir)
                    )

                duration = int(time.time() - start_time)
                minutes, seconds = divmod(duration, 60)
                print(f"Build completed in {minutes}:{seconds:02d}s using {num_proc} threads.")
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
            test_dir += "-release"
            exec_name = exec_name + ".exe"
        else:
            exec_name = "./" + exec_name

        test_dir = os.path.normpath(test_dir)
        exec_path = os.path.join(test_dir, exec_name)

        print(f"test_dir: {test_dir}")
        print(f"exec_name: {exec_name}")
        print(f"exec_path: {exec_path}")

        if not os.path.isdir(test_dir):
            raise FileNotFoundError(f"Directory not found: {test_dir}")
        if not os.path.isfile(exec_path):
            raise FileNotFoundError(f"Executable not found: {exec_path}")

        self.test_header(test_name)
        result = 0
        try:
            host_env = os.environ.copy()
            subprocess.check_call(exec_path, cwd=test_dir, shell=True, env=host_env)
        except subprocess.CalledProcessError as e:
            print(f"Error testing {test_name}!")
            result = e.returncode
        self.test_footer(result)
        if result:
            self.result = result
        return result

    def clean_rm(self, path):
        if self.run_clean:
            if os.path.exists(path):
                if os.path.isdir(path):
                    shutil.rmtree(path)
                    print(f"Directory removed: {path}")
                elif os.path.isfile(path):
                    os.remove(path)
                    print(f"File removed: {path}")
            else:
                raise FileNotFoundError(f"Path does not exist: {path}")

    def run(self):
        result = 0
        if self.run_build and self.project_dir:
            result = self.build_cmake(self.project_name, self.project_dir)
            if result:
                sys.exit(result)
        if self.run_test and self.project_dir:
            result = self.test_exec(self.project_name, self.project_dir, self.exec_name)
            if result:
                sys.exit(result)
        sys.exit(0)


def main(argv=None):
    args = parse_args(argv)
    bm = BuildTestManager(args)
    bm.run()


if __name__ == "__main__":
    main()
