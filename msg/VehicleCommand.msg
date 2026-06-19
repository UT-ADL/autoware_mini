std_msgs/Header header
uint8 mode
uint8 emergency

# Gear
uint8 gear
uint8 GEAR_NONE=0
uint8 GEAR_PARK=1
uint8 GEAR_REVERSE=2
uint8 GEAR_NEUTRAL=3
uint8 GEAR_DRIVE=4
uint8 GEAR_LOW=5

# Control
float32 speed
float32 acceleration
float32 steering_angle

# Turn signal
uint8 turn_signal
uint8 TURN_STRAIGHT=0
uint8 TURN_LEFT=1
uint8 TURN_RIGHT=2
uint8 TURN_HAZARD=3
