std_msgs/Header header

float32[3] base_lla 			# base position in lat-lon-altitude (deg, deg, m)
uint32 cycle_slip_count 	# number of cycle slips detected
uint32 rover_obs				# number of observations from rover (GPS, Glonass, Gallileo, Beidou, Qzs)
uint32 base_obs				# number of observations from base (GPS, Glonass, Gallileo, Beidou, Qzs)
uint32 rover_eph				# number of ephemeris messages from rover (GPS, Glonass, Gallileo, Beidou, Qzs)
uint32 base_eph				# number of ephemeris messages from rover (GPS, Glonass, Gallileo, Beidou, Qzs)
uint32 base_ant_count			# number of base station antenna position measurements

