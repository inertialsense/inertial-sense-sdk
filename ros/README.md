# Inertial Sense ROS

A ROS wrapper node implementation for the Inertial Sense IMX product line.

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
This ROS package, uses the inertial-sense-sdk as a submodule. Clone this package into the catkin workspace `src` folder, then pull the submodule.

```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
git clone https://github.com/inertialsense/inertial-sense-sdk
cd ..
catkin_make
```

For ROS to run properly, `devel/setup.bash` in the catkin workspace (catkin_ws) must be sourced to add necessary environment variables.  To understand more about this see the general [catkin documentation](http://wiki.ros.org/catkin).

```bash
source devel/setup.bash
```

You will need to run this command on every new shell you open to have access to the ROS commands, unless you add this line to your `~/.bashrc`.

### Important

**Firmware Version** - The IMX/uINS should be updated with the latest firmware found on the Inertial Sense [release page](https://github.com/inertialsense/inertial-sense-sdk/releases).  Download the appropriate `.hex` file and use the Inertial Sense EvalTool, CLTool, or SDK to upload the firmware.

**Dialout Group** - The user must be a member of the `dialout` group, or the user won't have access to the serial port.

## Execution

```bash
rosrun inertial_sense_ros inertial_sense_node
```

For instructions on changing parameter values and topic remapping from the command line while using `rosrun` refer to the [Remapping Arguments](http://wiki.ros.org/Remapping%20Arguments) page.  The following example shows how to set parameter using the ROS param server and run the inertial_sense_node:

```bash
rosparam set /inertial_sense_ros/navigation_dt_ms 16
rosparam set /inertial_sense_ros/msg/did_ins2/enable true
rosparam set /inertial_sense_ros/ref_lla "[40.25, -111.67, 1556.59]"
rosparam set /inertial_sense_ros/gps1_ant_xyz "[0.2, 0.0, 0.3]"
rosrun inertial_sense_ros inertial_sense_node
```

To set parameters and topic remapping from a launch file, refer to the [Roslaunch for Larger Projects](http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20tips%20for%20larger%20projects) page, or use one of the the sample launch files in this repository, `ros/launch/test_param_srv.launch` or  `ros/launch/test_YAML_params.launch`:

```bash
roslaunch inertial_sense_ros test_yaml_params.launch
```

```bash
roslaunch inertial_sense_ros test_param_srv.launch
```

## Timestamps

If GPS is available, all header timestamps are calculated with respect to the GPS clock but are translated into UNIX time to be consistent with the other topics in a ROS network.  If GPS is unvailable, then a constant offset between IMX time and system time is estimated during operation and is applied to IMU and INS message timestamps as they arrive.  There is often a small drift in these timestamps (on the order of a microsecond per second) due to variance in measurement streams and the difference between IMX and system clocks, however this is more accurate than stamping the measurements with ROS time as they arrive.  

Ideally there should be no jump in timestamps when GPS is first acquired because the timestamps should be identical.  However, due to inaccuracies in the system time, there will likely be a small jump in message timestamps after the first GPS fix.

## Topics

Topics are enabled and disabled using parameters.  By default, only the `ins` topic is published to save processor time in serializing unecessary messages.

- `odom_ins_ned` (nav_msgs/Odometry)
   - full 12-DOF measurements from onboard estimator in NED frame.
- `odom_ins_enu` (nav_msgs/Odometry)
   -  full 12-DOF measurements from onboard estimator in ENU frame.
- `odom_ins_ecef`(nav_msgs/Odometry)
   -  full 12-DOF measurements from onboard estimator in ECEF frame.
- `did_ins1` (inertial_sense_ros/did_ins1)
   -  Standard Inertial Sense [DID_INS_1](https://docs.inertialsense.com/user-manual/com-protocol/DID-descriptions/#did_ins1) Definition
- `did_ins2` (inertial_sense_ros/did_ins2)
   -  Standard Inertial Sense [DID_INS_2](https://docs.inertialsense.com/user-manual/com-protocol/DID-descriptions/#did_ins2) Definition
- `did_ins4` (inertial_sense_ros/did_ins4)
   -  Standard Inertial Sense [DID_INS_4](https://docs.inertialsense.com/user-manual/com-protocol/DID-descriptions/#did_ins_4) Definition
- `inl2_states` (inertial_sense_ros/INL2States)
   -  INS Extended Kalman Filter (EKF) states [DID_INL2_STATES](https://docs.inertialsense.com/user-manual/com-protocol/DID-descriptions/#did_inl2_states) Definition

- `imu`(sensor_msgs/Imu)
   -  Raw Imu measurements from IMU1 (NED frame)
- `pimu` (inertial_sense_ros/pimu)
   -  preintegrated coning and sculling integrals of IMU measurements
- `mag` (sensor_msgs/MagneticField)
   -  Raw magnetic field measurement from magnetometer 1
- `baro` (sensor_msgs/FluidPressure)
   -  Raw barometer measurements in kPa

- `NavSatFix`(sensor_msgs/NavSatFix)
   -  Standard ROS sensor_msgs/NavSatFix data
- `gps1`(inertial_sense_ros/gps1)
   -  GPS measurements from GPS1 receiver
- `gps2`(inertial_sense_ros/gps2)
   -  GPS measurements from GPS2 receiver
- `gps1/info`(inertial_sense_ros/gps1/info)
   -  satelite information and carrier noise ratio array for each satelite
- `gps2/info`(inertial_sense_ros/gps2/info)
   -  satelite information and carrier noise ratio array for each satelite
- `RTK_pos/info` (inertial_sense_ros/RTKInfo)
   -  information about RTK positioning status
- `RTK_pos/rel` (inertial_sense_ros/RTKRel)
   -  Relative measurement between RTK positioning base and rover
- `RTK_cmp/info` (inertial_sense_ros/RTKInfo)
   -  information about RTK compassing status
- `RTK_cmp/rel` (inertial_sense_ros/RTKRel)
   -  Relative measurement between RTK compassing moving base and rover

- `strobe_in` (std_msgs/Header)
   -  Timestamp of strobe in message header
- `diagnostics` (diagnostic_msgs/DiagnosticArray)
   -  Diagnostic message of RTK status.


__*Note: RTK positioning or RTK compassing mode must be enabled to stream any raw GPS data. Raw data can only be streamed from the onboard m8 receiver. To enable the onboard receiver change `gps1_type` to m8.__
- `<gps1_topic>/obs` (inertial_sense_ros/GNSSObservation)
    * Raw satellite observation (psuedorange and carrier phase)
- `<gps1_topic>/eph` (inertial_sense_ros/GNSSEphemeris)
    * Satellite Ephemeris for GPS and Galileo GNSS constellations
- `<gps1_topic>/geph`
    * Satellite Ephemeris for Glonass GNSS constellation

## Parameters

The Inertial Sense ROS parameters must contain the prefix `/inertial_sense_ros/...`  (i.e. `/inertial_sense_ros/navigation_dt_ms`).

- `~port` (string, default: "/dev/ttyACM0")
  - Serial port to connect to
- `~baudrate` (int, default: 921600)
  - baudrate of serial communication
- `~frame_id` (string, default "body")
  - frame id of all measurements
- `enable_log` (bool, default: false)
  - enable Inertial Sense Logger - logs PPD log in .dat format
- `~navigation_dt_ms` (int, default: Value retrieved from device flash configuration)
   - milliseconds between internal navigation filter updates (min=2ms/500Hz).  This is also determines the rate at which the topics are published.
- `~ioConfig` (int, default 39624800)
   - ioConfig bits in decimal format. Used for selection of GPS receiver type. See eIoConfig in data_sets.h

**Topic Configuration**

- `~msg/did_ins1/enable` (bool, default: false)
   - Flag to stream did_ins1 message
- `~msg/did_ins1/period` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~msg/did_ins2/enable` (bool, default: false)
   - Flag to stream did_ins2 message
- `~msg/did_ins2/period` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~msg/did_ins4/enable` (bool, default: false)
   - Flag to stream did_ins4 message
- `~msg/did_ins4/period` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~msg/odom_ins_ned/enable` (bool, default: true)
   - Flag to stream navigation solution in NED
- `~msg/odom_ins_ned/period` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~msg/odom_ins_enu/enable` (bool, default: false)
   - Flag to stream navigation solution in ENU
- `~msg/odom_ins_enu/period` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~msg/odom_ins_ecef/enable` (bool, default: false)
   - Flag to stream navigation solution in ECEF
- `~msg/odom_ins_ecef/period` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~stream_ins_covariance` (bool, default: false)
   - Flag to stream navigation covariance data in odometry messages
     __*Note__ - Data set is 176 bytes. Care should be taken to ensure sufficient bandwidth
- `~msg/inl2_states/enable` (bool, default: false)    
   -  Flag to stream INS2 state data
- `~msg/inls2_states/period` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~msg/imu/enable` (bool, default: false)
   - Flag to stream IMU measurements or not
- `~msg/imu/period` (int, default: 1)   
   - Configures period multiple of data set stream rate
- `~msg/baro/enable` (bool, default: false)   
   - Flag to stream baro or not
- `~msg/baro/period` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~msg/mag/enable` (bool, default: false)
   - Flag to stream magnetometer or not
- `~msg/mag/period` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~msg/pimu/enable` (bool, default: false)
   - Flag to stream preintegrated IMU or not
- `~msg/pimu/period` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~msg/gps1/enable`(bool, default: true)
   - Flag to stream GPS1
- `~msg/gps2/enable`(bool, default: false)
   - Flag to stream GPS2
- `~msg/gps1/period` (int, default: 1)
   - Configures GPS1 period multiple of data set stream rate
- `~msg/gps2/period` (int, default: 1)
   - Configures GPS2 period multiple of data set stream rate
- `~msg/gps1_info/enable`(bool, default: false)
   - Flag to stream GPS1 info messages
- `~msg/gps2_info/enable`(bool, default: false)
   - Flag to stream GPS2 info messages
- `~msg/gps1_info/period` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~msg/gps1_raw/enable` (bool, default: false)
   - Flag to stream GPS1 raw messages
- `~msg/gps2_raw/enable` (bool, default: false)
   - Flag to stream GPS2 raw messages
- `~msg/gps1_raw/period` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~msg/navsatfix/enable` (bool, default: false)
   - Flag to stream NavSatFix message
- `~msg/navsatfix/period` (int, default: 1)
   - Configures period multiple of data set stream rate.  Data is based on GPS2 if GPS1 is disabled.
- `~publishTf`(bool, default: true)
   - Flag to publish Tf transformations 'ins' to 'body_link'
- `~msg/diagnostics/enable` (bool, default: true)
   - Flag to stream diagnostics data
- `~msg/diagnostics/period` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~msg/rtk_pos/period` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~msg/rtk_cmp/period` (int, default: 1)
   - Configures period multiple of data set stream rate

## RTK Configuration

- `~rtk_compass` (bool, default: false)
   - Enables RTK rover mode (requires base corrections from an RTK base)
- `~rtk_base_USB` (bool, default: false)
   - Makes the connected uINS a RTK base station and enables the publishing of corrections our of Serial2 port
- `~rtk_base_serial` (bool, default: false)
   - Makes the connected uINS a RTK base station and enables the publishing of corrections out the USB port
- `~rtk_base_TCP` (bool, default: false)
   - Makes the connected uINS a RTK base station and creates a TCP server with which to publish corrections
- `~gnss_compass` (bool, default: false)
   - Uses both GPS antennas in a dual-GNSS configuration
- `~rtk_rover_radio_enable` (bool, default: false)
   - Enable radio on EVB2 for base corrections
- `~rtk_correction_protocol` (string, default: RTCM3)
   - Options are RTCM3 and UBLOX (for M8 receiver).  Rover and base must match.
- `~rtk_connection_attempt_limit` (string, default: RTCM3)
   - Number of times to attempt NTRIP connection
- `~rtk_connection_attempt_backoff` (string, default: RTCM3)
   - Sleep duration parameter. Sleep duration = attempt limit x attempt backoff
- `rtk_connectivity_watchdog_enabled` (bool default: false)
   - Data reception watchdog
- `rtk_connectivity_watchdog_timer_frequency` (float, default: 1)
   - frequency in which to check for traffic (secs)
- `rtk_data_transmission_interruption_limit` (float, default: 1)
   - time afterwhich connection will be reinitiated.

**TCP Configuration**
- `~rtk_server_IP` (string, default: 127.0.0.1)
  - If operating as base, attempts to create a TCP port on this IP for base corrections, if rover, connects to this IP for corrections.
- `~rtk_server_port` (int, default: 7777)
  - If operating as base, creates a TCP connection at this port for base corrections, if rover, connects to this port for corrections.

**NTRIP Configuration**

__*Note: These values must be clear for TCP configuration to work__
- `~rtk_server_mount` (string, default: "")
  - NTRIP mount point
- `~rtk_server_username` (string, default: "")
  - NTRIP username
- `~rtk_server_password` (string, default: "")
  - NTRIP password


**Sensor Configuration**
- `~ins_rotation` (vector(3), default: {0, 0, 0})
   -  The roll, pitch, yaw rotation from the INS frame to the output frame
- `~ins_offset` (vector(3), default: {0, 0, 0})
   -  The NED translation vector between the INS frame and the output frame (wrt output frame)
- `~gps1_type` (string, default: "F9P")
   -  Which receiver type: "F9P" or "M8"
- `gps2_type` (string, default: "F9P")
   -  Which receiver type: "F9P" or "M8"
- `~msg/gps1/topic` (string, default: "gps1")
   -  ROS topic name of GPS1 stream
- `~msg/gps2/topic` (string, default: "gps2")
   -  ROS topic name of GPS1 stream
- `~gps1_ant_xyz` (vector(3), default: {0, 0, 0})
   -  The NED translation vector between the INS frame and the GPS 1 antenna (wrt INS frame)
- `~gps2_ant_xyz` (vector(3), default: {0, 0, 0})
   -  The NED translation vector between the INS frame and the GPS 2 antenna (wrt INS frame)
- `~ref_lla` (vector(3), default: {0, 0, 0})
   -  The Reference longitude, latitude and altitude for NED calculation in degrees, degrees and meters (use the `set_refLLA` service to update this automatically)
- `~gpsTimeUserDelay` (float, default: 0)
   -  GPS update time delay
- `~mag_declination` (float, default: 0.20007290992)
   -  The mag_declination of earth's magnetic field (radians)
- `~dynamic_model` (int, default: 8)
   -  Dynamic model used in internal filter of uINS.
      -  0 = portable
      -  2 = stationary
      -  3 = pedestrian
      -  4 = ground vehicle
      -  5 = sea
      -  6 = airborne 1G
      -  7 = airborne 2G
      -  8 = airborne 4G
      -  9 = wrist

**NMEA Output Configuration - NOT CURRENTLY SUPPORTED**
- `~ser1_baud_rate` (int, default: 921600)
   -  baud rate for serial1 port used for external NMEA messaging (located on H6-5) [serial port hardware connections](http://docs.inertialsense.com/user-manual/Setup_Integration/hardware_integration/#pin-definition)
- `~NMEA_rate` (int, default: 0)
   -  Rate to publish NMEA messages
- `~NMEA_configuration` (int, default: 0x00)
   -  bitmask to enable NMEA messages (bitwise OR to enable multiple message streams).
      -  GPGGA = 0x01
      -  GPGLL = 0x02
      -  GPGSA = 0x04
      -  GPRMC = 0x08
- `~NMEA_ports` (int, default: 0x00)
   -  bitmask to enable NMEA message on serial ports (bitwise OR to enable both ports) 
      -  Ser0 (USB/H4-4)  = 0x01 
      -  Ser1 (H6-5) = 0x02

## Services
- `single_axis_mag_cal` (std_srvs/Trigger)
  - Put INS into single axis magnetometer calibration mode.  This is typically used if the uINS is rigidly mounted to a heavy vehicle that will not undergo large roll or pitch motions, such as a car. After this call, the uINS must perform a single orbit around one axis (i.g. drive in a circle) to calibrate the magnetometer [more info](https://docs.inertialsense.com/user-manual/reference/magnetometer/)
- `multi_axis_mag_cal` (std_srvs/Trigger)
  - Put INS into multi axis magnetometer calibration mode.  This is typically used if the uINS is not mounted to a vehicle, or a lightweight vehicle such as a drone.  Simply rotate the uINS around all axes until the light on the uINS turns blue [more info](https://docs.inertialsense.com/user-manual/reference/magnetometer/)
- `firmware_update` (inertial_sense_ros/FirmwareUpdate)
  - Updates firmware to the `.hex` file supplied (use absolute filenames)
- `set_refLLA_current` (std_srvs/Trigger)
  - Takes the current estimated position and sets it as the `refLLA`.  Use this to set a base position after a survey, or to zero out the `ins` topic.1
- `set_refLLA_value` (std_srvs/Trigger)
  - Sets `refLLA` to the values passed as service arguments of type float64[3].  Use this to set refLLA to a known value.
