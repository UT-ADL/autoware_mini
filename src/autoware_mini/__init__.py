import warnings
import rospy

# Log warnings in ROS
def ros_warning_handler(message, category, filename, lineno, file=None, line=None):
    rospy.logwarn_throttle(5, f"{filename}:{lineno}: {category.__name__}: {message}")

warnings.showwarning = ros_warning_handler
