std_msgs/Header header                       # GPS time of week (since Sunday morning) in seconds
geometry_msgs/Quaternion quat_ecef   # Quaternion body rotation with respect to ECEF
geometry_msgs/Vector3 vel_ecef       # (m/s) Velocity in ECEF frame
geometry_msgs/Vector3 pos_ecef       # (m) Position in ECEF frame
geometry_msgs/Vector3 gyro_bias      # (rad/s) Gyro bias
geometry_msgs/Vector3 accel_bias     # (m/s^2) Accelerometer bias
float32 baro_bias                    # (m) Barometer bias
float32 mag_dec                      # (rad) Magnetic declination
float32 mag_inc                      # (rad) Magnetic inclination
