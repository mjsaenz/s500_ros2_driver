# Message to send s500 backscatter data, capturing all fields from distance2 ping-protocol message
# https://docs.bluerobotics.com/ping-protocol/pingmessage-s500/#1223-distance2

std_msgs/Header header

# default values zeroed out to signal when valid profile has not been received
# fields presented in same order as official documentation

uint32 ping_distance_mm 0 # distance of most recent ping
uint32 averaged_distance_mm 0 # average distance over last 20 pings
# uint16 reserved
uint8 ping_confidence 0
uint8 average_distance_confidence 0
uint32 timestamp_msec 0
