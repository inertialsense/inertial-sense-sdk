Header header
uint32 week                             # GPS number of weeks since January 6th, 1980 
int8 num_sat                            # Number of satellites used in solution
uint32 status                           # Fix type and status flags.  See GPS_STATUS_FIX_... and GPS_STATUS_FLAGS_... in sdk data_sets.h
int32 cno                               # Average of all non-zero satellite carrier to noise ratios, signal strengths (dBHz)
float64 latitude                        # Latitude (degrees)
float64 longitude                       # Longitude (degrees)
float64 altitude                        # Height above ellipsoid (not MSL) (m)
geometry_msgs/Vector3 posEcef           # Position (m) in ECEF
geometry_msgs/Vector3 velEcef           # Velocity (m/s) in ECEF
float32 hMSL                            # Height above MSL
float32 hAcc                            # Horizontal accuracy
float32 vAcc                            # Vertical accuracy
float32 sAcc                            # Speed accuracy (m/s)
float32 pDop                            # Position Dilution of Precision (m)	

