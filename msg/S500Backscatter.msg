# Message to send s500 backscatter data, capturing all fields from profile6_t ping-protocol message
# https://docs.bluerobotics.com/ping-protocol/pingmessage-s500/#1308-profile6_t

std_msgs/Header header

# default values zeroed out to signal when valid profile has not been received

uint32 ping_number 0
uint32 start_mm 0
uint32 length_mm 0
uint32 start_ping_hz 0
uint32 end_ping_hz 0
uint32 adc_sample_hz 0
uint32 timestamp_msec 0
float32 ping_duration_sec 0
float32 analog_gain 0
float32 max_pwr_db 0 # get_profile6_t() dictionary key is "max_pwr"
float32 min_pwr_db 0 # get_profile6_t() dictionary key is "min_pwr"
float32 step_db 0
float32 is_db 0
uint8 gain_index 0
uint8 decimation 0
uint16 num_results 0
uint16[] pwr_results
