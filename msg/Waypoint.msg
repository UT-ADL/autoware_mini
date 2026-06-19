geometry_msgs/Point32 position
float32 speed
float32 speed_limit

# path width
geometry_msgs/Point32 left_boundary_point
geometry_msgs/Point32 right_boundary_point

# lane boundary type
uint8 left_boundary_type
uint8 right_boundary_type
uint8 VIRTUAL=0
uint8 SOLID=1
uint8 SOLID_SOLID=2
uint8 DASHED=3
uint8 DASHED_SOLID=4
uint8 SOLID_DASHED=5
uint8 KEEPOUT=6
uint8 PEDESTRIAN_MARKING=7
uint8 CURBSTONE=8
uint8 ROAD_BORDER=9

# lanechange_state
uint8 lanechange_state
bool priority

# turn signal
uint8 turn_signal
uint8 TURN_STRAIGHT=0
uint8 TURN_LEFT=1
uint8 TURN_RIGHT=2

# stop line
int32 stop_line_id
uint8 stop_line_type
uint8 STOP_LINE_NONE=0
uint8 STOP_LINE_YIELD=1
uint8 STOP_LINE_YIELD_STOP=2
uint8 STOP_LINE_YIELD_MANUAL=3
uint8 STOP_LINE_YIELD_RIGHT=4
uint8 STOP_LINE_PRIORITY=5
uint8 SPEED_BUMP=6
