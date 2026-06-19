std_msgs/Header header

# Type of the stop lines in the statuses
uint8 type
uint8 TRAFFIC_LIGHT = 0
uint8 YIELD = 1
uint8 YIELD_STOP = 2
uint8 YIELD_MANUAL = 3
uint8 RIGHT_OF_WAY = 4

StopLineStatus[] statuses
