import os
import shutil
import subprocess
import sys
from pathlib import Path
import build_test_manager

sdk_dir = Path(__file__).resolve().parent.parent
bm = build_test_manager.BuildTestManager()

###############################################################################
#  Builds and Tests
###############################################################################

bm.build_cmake("IS_SDK_lib", sdk_dir)
bm.build_cmake("cltool", sdk_dir / "cltool")

# build_header "LogInspector"
# ./build_log_inspector.sh
# build_footer $?

bm.build_cmake("SDK_Unit_Tests", sdk_dir / "tests", "IS-SDK_unit-tests")
bm.build_cmake("SDK_Examples", sdk_dir / "ExampleProjects")

###############################################################################
#  Summary
###############################################################################

bm.build_summary()

# echo_build "=========================================="
# echo_build " BUILD RESULTS:"
# echo_build "=========================================="
# if [ -n "$BUILD_SUCCESS" ]
# then
#     echo_green "[PASSED]: $BUILD_SUCCESS"
# fi
# if [ -n "$BUILD_FAILURES" ]
# then
#     echo_red "[FAILED]: $BUILD_FAILURES"
# fi
# echo 

# if [ -n "$RELEASE_BUILD" ]; then 
#     echo_yellow_title "Generated: RELEASE ${RELEASE_NAME}"
# fi

# popd > /dev/null



