uint8 GPS_STATUS_FIX_3D                   = 1
uint8 GPS_STATUS_FIX_RTK_SINGLE           = 2
uint8 GPS_STATUS_FIX_RTK_FLOAT            = 3
uint8 GPS_STATUS_FIX_RTK_FIX              = 4
uint8 GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD   = 5



Header header
float32 differential_age 				# Age of differential (seconds)
float32 ar_ratio 						# Ambiguity resolution ratio factor for validation
uint8 eGpsNavFixStatus                  # GPS navigation fix type
geometry_msgs/Vector3 vector_base_to_rover 	# Vector to base (m) in ECEF - If Compassing enabled, this is the 3-vector from antenna 2 to antenna 1
float32 distance_base_to_rover 				# Distance to Base (m)
float32 heading_base_to_rover 				# Angle from north to vectorToBase in local tangent plane. (rad)