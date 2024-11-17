import os
import shutil
import subprocess
import sys
from pathlib import Path
import build_test_manager
import build_log_inspector

sdk_dir = Path(__file__).resolve().parent.parent
bm = build_test_manager.BuildTestManager()

###############################################################################
#  Builds and Tests
###############################################################################

bm.build_cmake("IS_SDK_lib", sdk_dir)
bm.build_cmake("cltool", sdk_dir / "cltool")

bm.build_callback("LogInspector", build_log_inspector.run_build)

bm.build_cmake("SDK_Unit_Tests", sdk_dir / "tests")
bm.build_cmake("SDK_Examples", sdk_dir / "ExampleProjects")

# Additional files to remove (if -c or --clean provided)
bm.clean_rm(sdk_dir / "libInertialSenseSDK.a")
bm.clean_rm(sdk_dir / "CMakeFiles")
bm.clean_rm(sdk_dir / "cmake_install.cmake")
bm.clean_rm(sdk_dir / "Makefile")
bm.clean_rm(sdk_dir / "CMakeCache.txt")


###############################################################################
#  Tests (run using `-t` or `--test`)
###############################################################################

bm.test_exec("IS-SDK Unit Tests",    sdk_dir / "tests", "IS-SDK_unit-tests")


###############################################################################
#  Summary
###############################################################################

bm.print_summary()


