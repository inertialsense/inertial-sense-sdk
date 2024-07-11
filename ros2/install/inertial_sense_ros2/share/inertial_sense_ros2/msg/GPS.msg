std_msgs/Header header
uint32 week                             # GPS number of weeks since January 6th, 1980 
int8 num_sat                            # Number of satellites used in solution
uint32 status                           # Fix type and status flags.  See GPS_STATUS_FIX_... and GPS_STATUS_FLAGS_... in sdk data_sets.h
int32 cno                               # Average of all non-zero satellite carrier to noise ratios, signal strengths (dBHz)
float64 latitude                        # Latitude (degrees)
float64 longitude                       # Longitude (degrees)
float64 altitude                        # Height above ellipsoid (not MSL) (m)
geometry_msgs/Vector3 pos_ecef           # Position (m) in ECEF
geometry_msgs/Vector3 vel_ecef           # Velocity (m/s) in ECEF
float32 hmsl                            # Height above MSL
float32 hacc                            # Horizontal accuracy
float32 vacc                            # Vertical accuracy
float32 sacc                            # Speed accuracy (m/s)
float32 pdop                            # Position Dilution of Precision (m)	

