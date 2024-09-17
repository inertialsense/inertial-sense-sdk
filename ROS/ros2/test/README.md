# Inertial Sense ROS2 Testing


---
### ***************  NOTICE  ***************
The source code in this directory and its subdirectories is provided "as-is", without warranty or guarantee of any kind.

Inertial Sense acknowledges that the functionality provided in this ROS component, is a limited subset of functionality provided by the
C++ SDK component, contained within this GitHub repository. Any functionality provided by the C++ SDK, which is not available through this
ROS component, is the responsibility of the customer and/or user to implement as necessary.

While we aim to make every effort to maintain and improve (and validate through tests) this ROS wrapper software component, it is done at
our leisure, and as time permits relative to our other internal business priorities. As this component is provided as an "open-source"
project, with all source-code publicly available, it is our hope that this component will be "community supported". In this end, we at
Inertial Sense actively encourage code contributions from customers and other users who wish to extend the functionality, or resolve defects
that exist within the code, through the use of GitHub's Fork and Pull-Request mechanisms.

### Disclaimer of Software Warranty.

INERTIAL SENSE LICENSES THE SOFTWARE AND ASSOCIATED SOURCE CODE UNDER THIS DIRECTORY "AS IS", AND MAKES NO EXPRESS OR IMPLIED WARRANTY OF
ANY KIND. INERTIAL SENSE SPECIFICALLY DISCLAIMS ALL INDIRECT OR IMPLIED WARRANTIES TO THE FULL EXTENT ALLOWED BY APPLICABLE LAW, INCLUDING
WITHOUT LIMITATION ALL IMPLIED WARRANTIES OF, NON-INFRINGEMENT, MERCHANTABILITY, TITLE OR FITNESS FOR ANY PARTICULAR PURPOSE. NO ORAL OR
WRITTEN INFORMATION OR ADVICE GIVEN BY INERTIAL SENSE, ITS AGENTS OR EMPLOYEES SHALL CREATE A WARRANTY.

THE USE OF THIS SOFTWARE IS AT YOUR OWN RISK, AND YOU ASSUME ALL RESPONSIBILITY FOR ANY LOSS, DAMAGE, OR OTHER HARM THAT MAY RESULT FROM
THE USE OF THIS SOFTWARE. INERTIAL SENSE SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL OR CONSEQUENTIAL DAMAGES ARISING
OUT OF OR IN ANY WAY CONNECTED WITH THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

---

## Setup
Use `colcon build` from the `ros2_ws` directory to build the ros2 package, then use `ros2 run inertial_sense_ros2 test_new_target` to run the tests locally. Otherwise, these tests will run in GitHub actions everytime a commit is made
to the `imx` repository.

## Files
`gtest_helpers.h`: This file contains parameters necessary for running gtest (Google Test) interface for ROS2.
 
`setup_tests.sh`: Shell script to set up the tests. This is still a work in progress.

 `test_basic_unit_tests.cpp`: Tests RTK and topic helper functionality. Provides a `main` function that initializes ROS2 to run the tests.
Cannot be included in CMakeLists.txt at the same time as `test_main.cpp`

`test_client_reconnect.cpp`: Tests to make sure RTK connection is available to the device. Provides error messages if
unsuccessful.

`test_communications.cpp`: Tests NavSatFix functionality of the connected device by including a subscriber for
NavSatFix.

`test_main.cpp`: Tests PIMU and gps_ins_time_sync messages and includes multiple subscribers to test this.
Cannot be included in CMakeLists.txt at the same time as `test_main.cpp`.

`test_main.h`: Helper file for `test_main.cpp`
