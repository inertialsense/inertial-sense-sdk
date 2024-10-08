std_msgs/Header header
GTime time              # time of observation
uint8 sat 		# satellite number
uint8 rcv 		# receiver number
uint8 snrr 		# Signal Strength (0.25 dBHz)
uint8 lli 		# Loss-of-Lock Indicator (bit1=loss-of-lock, bit2=half-cycle-invalid)
uint8 code 		# code indicator (BeiDou: CODE_L1I, Other: CODE_L1C )
uint8 qual_l 	# Estimated carrier phase measurement standard deviation (0.004 cycles)
uint8 qual_p 	# Estimated pseudorange measurement standard deviation (0.01 m)
float64 l      	# observation data carrier-phase (cycle)
float64 p      	# observation data pseudorange (m)
float32 d      	# observation data doppler frequency (0.002 Hz)
