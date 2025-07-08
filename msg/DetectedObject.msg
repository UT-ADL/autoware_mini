uint32                          id
string                          label
float32                         score   #Score as defined by the detection, Optional
std_msgs/ColorRGBA              color   # Define this object specific color
bool                            valid   # Defines if this object is valid, or invalid as defined by the filtering

################ 3D BB
geometry_msgs/Point32           centroid    # mass center of lidar points for better tracking
geometry_msgs/Point32           center      # center of the bounding box and origin for object dimensions
geometry_msgs/Vector3           dimensions
#geometry_msgs/Vector3           variance
geometry_msgs/Vector3           velocity
geometry_msgs/Vector3           acceleration
float32                         heading

float32[]                       convex_hull
PathArray                       candidate_trajectories

bool                            position_reliable
bool                            velocity_reliable
bool                            acceleration_reliable

############### Indicator information
#uint8                          indicator_state # INDICATOR_LEFT = 0, INDICATOR_RIGHT = 1, INDICATOR_BOTH = 2, INDICATOR_NONE = 3

############### Behavior State of the Detected Object
#uint8                           behavior_state # FORWARD_STATE = 0, STOPPING_STATE = 1, BRANCH_LEFT_STATE = 2, BRANCH_RIGHT_STATE = 3, YIELDING_STATE = 4, ACCELERATING_STATE = 5, SLOWDOWN_STATE = 6
