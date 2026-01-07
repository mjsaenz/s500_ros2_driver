# Message to send s500 backscatter data, capturing all fields from profile6_t ping-protocol message
# https://docs.bluerobotics.com/ping-protocol/pingmessage-s500/#1308-profile6_t

std_msgs/Header header

# default values zeroed out to signal when valid profile has not been received
# fields presented in same order as official documentation

uint32 ping_number 0
uint32 start_mm 0
uint32 length_mm 0
uint32 start_ping_hz 0
uint32 end_ping_hz 0
uint32 adc_sample_hz 0
uint32 timestamp_msec 0
# uint32 spare2
float32 pulse_duration_sec 0
float32 analog_gain 0
float32 max_pwr_db 0 
float32 min_pwr_db 0 
float32 this_ping_depth_m 0
float32 smooth_depth_m 0
# float32 fspare2
uint8 ping_depth_measurement_confidence 0
uint8 gain_index 0
uint8 decimation 0
uint8 smooth_depth_measurement_confidence 0
uint16 num_results 0
uint16[] pwr_results
