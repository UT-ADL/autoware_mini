# Stop line id to which this traffic light result belongs to
int32 stop_line_id

# Traffic light id
int32 traffic_light_id

# The (aggregated) status from the tlr nodes or status from checker nodes
# Red and Yellow states are both treated as STOP
uint8 status
uint8 STATUS_STOP = 0
uint8 STATUS_GO = 1
uint8 STATUS_UNKNOWN = 2
uint8 STATUS_MISSING = 3

# This string is visualized on top of stop line
string status_text