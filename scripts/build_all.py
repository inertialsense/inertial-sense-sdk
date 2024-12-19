from pathlib import Path
import build_manager
import build_log_inspector
import sys

sdk_dir = Path(__file__).resolve().parent.parent
bm = build_manager.BuildTestManager()


###############################################################################
#  Builds
###############################################################################

bm.build_cmake("IS_SDK_lib", sdk_dir)
bm.build_cmake("cltool", sdk_dir / "cltool")
bm.build_callback("LogInspector", build_log_inspector.run_build)
bm.build_cmake("SDK_Unit_Tests", sdk_dir / "tests")
bm.build_cmake("SDK_Examples", sdk_dir / "ExampleProjects")


###############################################################################
#  Tests (run using `-t` or `--test`)
###############################################################################

bm.test_exec("IS-SDK Unit Tests",    sdk_dir / "tests", "IS-SDK_unit-tests")


###############################################################################
#  Summary
###############################################################################

bm.print_summary()

# Return error code
sys.exit(bm.result)
