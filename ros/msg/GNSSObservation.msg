std_msgs/Header header
GTime       time        # time of observation
uint8       sat         # satellite number
uint8       rcv         # receiver number
uint16[2]   SNR         # Signal Strength (0.25 dBHz)
uint8[2]    LLI         # Loss-of-Lock Indicator (bit1=loss-of-lock, bit2=half-cycle-invalid)
uint8[2]    code        # code indicator (BeiDou: CODE_L1I, Other: CODE_L1C )
float64[2]  L           # Observation data carrier-phase (cycle). The carrier phase initial ambiguity is initialized using an approximate value to make the magnitude of the phase close to the pseudorange measurement. Clock resets are applied to both phase and code measurements in accordance with the RINEX specification.
float64[2]  P           # Observation data pseudorange (m). GLONASS inter frequency channel delays are compensated with an internal calibration table
float32[2]  D           # Observation data Doppler measurement (positive sign for approaching satellites) (Hz)
int32       timevalid   # time is valid (Valid GNSS fix) for time mark
GTime       eventime    # time of event (GPST)
uint8[2]    Lstd        # stdev of carrier phase (0.004 cycles)
uint8[2]    Pstd        # stdev of pseudorange (0.01*2^(n+5) meters)
uint8       freq        # GLONASS frequency channel (0-13)
uint8       reserved    # 
