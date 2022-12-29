# Inertial Sense ROS

A ROS wrapper node implementation for the Inertial Sense IMX product line.

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
rosparam set /inertial_sense_ros/stream_DID_INS_2 true
rosparam set /inertial_sense_ros/GPS_ref_lla "[40.25, -111.67, 1556.59]"
rosparam set /inertial_sense_ros/GPS_ant1_xyz "[0.2, 0.0, 0.3]"
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
- `DID_INS_1`(inertial_sense_ros/DID_INS_1)
   -  Standard Inertial Sense [DID_INS_1](https://docs.inertialsense.com/user-manual/com-protocol/DID-descriptions/#did_ins_1) Definition
- `DID_INS_2`(inertial_sense_ros/DID_INS_2)
   -  Standard Inertial Sense [DID_INS_2](https://docs.inertialsense.com/user-manual/com-protocol/DID-descriptions/#did_ins_2) Definition
- `DID_INS_4`(inertial_sense_ros/DID_INS_4)
   -  Standard Inertial Sense [DID_INS_4](https://docs.inertialsense.com/user-manual/com-protocol/DID-descriptions/#did_ins_4) Definition
- `imu`(sensor_msgs/Imu)
   -  Raw Imu measurements from IMU1 (NED frame)
- `gps`(inertial_sense_ros/GPS)
   -  unfiltered GPS measurements from onboard GPS unit
- `gps/info`(inertial_sense_ros/GPSInfo)
   -  satelite information and carrier noise ratio array for each sattelite
- `NavSatFix`(sensor_msgs/NavSatFix)
   -  Standard ROS sensor_msgs/NavSatFix data
- `mag` (sensor_msgs/MagneticField)
   -  Raw magnetic field measurement from magnetometer 1
- `baro` (sensor_msgs/FluidPressure)
   -  Raw barometer measurements in kPa
- `preint_imu` (inertial_sense_ros/DThetaVel)
   -  preintegrated coning and sculling integrals of IMU measurements
- `RTK_pos/info` (inertial_sense_ros/RTKInfo)
   -  information about RTK status
- `RTK_pos/rel` (inertial_sense_ros/RTKRel)
   -  Relative measurement between RTK base and rover
- `inl2_states` (inertial_sense_ros/INL2States)
   -  INS Extended Kalman Filter (EKF) states [DID_INL2_STATES](https://docs.inertialsense.com/user-manual/com-protocol/DID-descriptions/#did_inl2_states) Definition
- `diagnostics` (diagnostic_msgs/DiagnosticArray)
   -  Diagnostic message of RTK status.
- `strobe_time` (std_msgs/Header)
   -  Timestamp of strobe in message header


__*Note: RTK positioning or RTK compassing mode must be enabled to stream any raw GPS data. Raw data can only be streamed from the onboard m8 receiver. To enable the onboard receiver change `GPS1_type` to m8.__
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

- `~stream_DID_INS_1` (bool, default: false)
   - Flag to stream DID_INS_1 message
- `~ins1_period_multiple` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~stream_DID_INS_2` (bool, default: false)
   - Flag to stream DID_INS_2 message
- `~ins2_period_multiple` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~stream_DID_INS_4` (bool, default: false)
   - Flag to stream DID_INS_4 message
- `~ins4_period_multiple` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~stream_odom_ins_ned` (bool, default: true)
   - Flag to stream navigation solution in NED
- `~odom_ins_ned_period_multiple` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~stream_odom_ins_enu` (bool, default: false)
   - Flag to stream navigation solution in ENU
- `~odom_ins_enu_period_multiple` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~stream_odom_ins_ecef` (bool, default: false)
   - Flag to stream navigation solution in ECEF
- `~odom_ins_ecef_period_multiple` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~stream_covariance_data` (bool, default: false)
   - Flag to stream navigation covariance data in odometry messages

     __*Note__ - Data set is 176 bytes. Care should be taken to ensure sufficient bandwidth
- `~stream_INL2_states` (bool, default: false)    
   -  Flag to stream INS2 state data
- `~INL2_states_period_multiple` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~stream_IMU` (bool, default: false)
   - Flag to stream IMU measurements or not
- `~imu_period_multiple` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~stream_baro` (bool, default: false)
   - Flag to stream baro or not
- `~baro_period_multiple` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~stream_mag` (bool, default: false)
   - Flag to stream magnetometer or not
- `~mag_period_multiple` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~stream_preint_IMU` (bool, default: false)
   - Flag to stream preintegrated IMU or not
- `~preint_imu_period_multiple` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~stream_GPS1`(bool, default: false)
   - Flag to stream GPS1
- `~stream_GPS2`(bool, default: false)
   - Flag to stream GPS2
- `~gps1_period_multiple` (int, default: 1)
   - Configures GPS1 period multiple of data set stream rate
- `~gps2_period_multiple` (int, default: 1)
   - Configures GPS2 period multiple of data set stream rate
- `~stream_GPS1_info`(bool, default: false)
   - Flag to stream GPS1 info messages
- `~stream_GPS2_info`(bool, default: false)
   - Flag to stream GPS2 info messages
- `~gps_info_period_multiple` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~stream_GPS1_raw` (bool, default: false)
   - Flag to stream GPS1 raw messages
- `~stream_GPS2_raw` (bool, default: false)
   - Flag to stream GPS2 raw messages
- `~gps_raw_period_multiple` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~stream_NavSatFix` (bool, default: false)
   - Flag to stream NavSatFix message
- `~NavSatFix_period_multiple` (int, default: 1)
   - Configures period multiple of data set stream rate.  Data is based on GPS2 if GPS1 is disabled.
- `~publishTf`(bool, default: true)
   - Flag to publish Tf transformations 'ins' to 'body_link'
- `~stream_diagnostics` (bool, default: true)
   - Flag to stream diagnostics data
- `~diagnostics_period_multiple` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~RTK_pos_period_multiple` (int, default: 1)
   - Configures period multiple of data set stream rate
- `~RTK_cmp_period_multiple` (int, default: 1)
   - Configures period multiple of data set stream rate

## RTK Configuration

- `~RTK_rover` (bool, default: false)
   - Enables RTK rover mode (requires base corrections from an RTK base)
- `~RTK_base_USB` (bool, default: false)
   - Makes the connected uINS a RTK base station and enables the publishing of corrections our of Serial2 port
- `~RTK_base_serial` (bool, default: false)
   - Makes the connected uINS a RTK base station and enables the publishing of corrections out the USB port
- `~RTK_base_TCP` (bool, default: false)
   - Makes the connected uINS a RTK base station and creates a TCP server with which to publish corrections
- `~GNSS_Compass` (bool, default: false)
   - Uses both GPS antennas in a dual-GNSS configuration
- `~RTK_rover_radio_enable` (bool, default: false)
   - Enable radio on EVB2 for base corrections
- `~RTK_correction_protocol` (string, default: RTCM3)
   - Options are RTCM3 and UBLOX (for M8 receiver).  Rover and base must match.
- `~RTK_connection_attempt_limit` (string, default: RTCM3)
   - Number of times to attempt NTRIP connection
- `~RTK_connection_attempt_backoff` (string, default: RTCM3)
   - Sleep duration parameter. Sleep duration = attempt limit x attempt backoff
- `RTK_connectivity_watchdog_enabled` (bool default: false)
   - Data reception watchdog
- `RTK_connectivity_watchdog_timer_frequency` (float, default: 1)
   - frequency in which to check for traffic (secs)
- `RTK_data_transmission_interruption_limit` (float, default: 1)
   - time afterwhich connection will be reinitiated.

**TCP Configuration**
- `~RTK_server_IP` (string, default: 127.0.0.1)
  - If operating as base, attempts to create a TCP port on this IP for base corrections, if rover, connects to this IP for corrections.
- `~RTK_server_port` (int, default: 7777)
  - If operating as base, creates a TCP connection at this port for base corrections, if rover, connects to this port for corrections.

**NTRIP Configuration**

__*Note: These values must be clear for TCP configuration to work__
- `~RTK_server_mount` (string, default: "")
  - NTRIP mount point
- `~RTK_server_username` (string, default: "")
  - NTRIP username
- `~RTK_server_password` (string, default: "")
  - NTRIP password


**Sensor Configuration**
- `~INS_rpy_radians` (vector(3), default: {0, 0, 0})
   -  The roll, pitch, yaw rotation from the INS frame to the output frame
- `~INS_xyz` (vector(3), default: {0, 0, 0})
   -  The NED translation vector between the INS frame and the output frame (wrt output frame)
- `~GPS1_type` (string, default: "F9P")
   -  Which receiver type: "F9P" or "M8"
- `~GPS1_topic` (string, default: "gps1")
   -  ROS topic name of GPS1 stream
- `GPS2_type` (string, default: "F9P")
   -  Which receiver type: "F9P" or "M8"
- `~GPS2_topic` (string, default: "gps2")
   -  ROS topic name of GPS1 stream
- `~GPS_ant1_xyz` (vector(3), default: {0, 0, 0})
   -  The NED translation vector between the INS frame and the GPS 1 antenna (wrt INS frame)
- `~GPS_ant2_xyz` (vector(3), default: {0, 0, 0})
   -  The NED translation vector between the INS frame and the GPS 2 antenna (wrt INS frame)
- `~GPS_ref_lla` (vector(3), default: {0, 0, 0})
   -  The Reference longitude, latitude and altitude for NED calculation in degrees, degrees and meters (use the `set_refLLA` service to update this automatically)
- `~gpsTimeUserDelay` (float, default: 0)
   -  GPS update time delay
- `~declination` (float, default: 0.20007290992)
   -  The declination of earth's magnetic field (radians)
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

**ASCII Output Configuration - NOT CURRENTLY SUPPORTED**
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
