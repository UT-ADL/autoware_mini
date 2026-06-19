std_msgs/Header header

# Powertrain
uint8 drivemode
uint8 steeringmode
uint8 MODE_MANUAL=0
uint8 MODE_AUTO=1

uint8 gear
uint8 GEAR_NONE=0
uint8 GEAR_PARK=1
uint8 GEAR_REVERSE=2
uint8 GEAR_NEUTRAL=3
uint8 GEAR_DRIVE=4
uint8 GEAR_LOW=5

float32 speed # vehicle speed [m/s]
float32 drivepedal
float32 brakepedal

float32 angle # vehicle steering (tire) angle [rad]

# Turn signal
uint8 turn_signal
uint8 TURN_STRAIGHT=0
uint8 TURN_LEFT=1
uint8 TURN_RIGHT=2
uint8 TURN_HAZARD=3
