
import rospy
import numpy as np
import matplotlib.pyplot as plt
from autoware_msgs.msg import Lane
from geometry_msgs.msg import PoseStamped

from scipy import interpolate



class LocalPlanner:

    def __init__(self):

        # Parameters
        self.waypoint_interval = rospy.get_param("~waypoint_interval", 1.0)

        # Internal variables

        # Publishers

        # Subscribers
        self.path_sub = rospy.Subscriber('/planning/path', Lane, self.path_callback, queue_size=1)
        self.current_pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)


    def path_callback(self, lane):
        
        waypoints_array = np.array([[wp.pose.pose.position.x, wp.pose.pose.position.y, wp.pose.pose.position.z, wp.wpstate.steering_state] for wp in lane.waypoints])        
        self.global_path = interpolate_using_distance(waypoints_array, self.waypoint_interval)

    def current_pose_callback(self, msg):

        # local_path is extracted from global_path in every step if it is fast using array slicing
        
        # local path

        # check dist from end of local_path - update local path if needed


        pass


    def run(self):
        rospy.spin()


def interpolate_using_distance(waypoints_array, waypoint_interval):

        # extract into arrays
        x_path = waypoints_array[:,0]
        y_path = waypoints_array[:,1]
        z_path = waypoints_array[:,2]
        blinker = waypoints_array[:,3]

        # TODO speeds, curvature and adjsut speed based on that.

        # distance differeneces between points
        x_diff = np.diff(x_path)
        y_diff = np.diff(y_path)
        distances = np.cumsum(np.sqrt(x_diff**2 + y_diff**2))
        # add 0 to the beginning of the array
        distances = np.insert(distances, 0, 0)
        new_distances = np.linspace(0, distances[-1], num=int(distances[-1] / waypoint_interval))

        x_new = np.interp(new_distances, distances, x_path)
        y_new = np.interp(new_distances, distances, y_path)
        z_new = np.interp(new_distances, distances, z_path)
        # test blinkers with equal weights?
        blinker_new = np.interp(new_distances, distances, blinker)

        # debug visualization
        debug_visualize(x_path, y_path, z_path, blinker, x_new, y_new, z_new, blinker_new, distances, new_distances)

        # Stack arrays for faster accessing - call it global path array 


def debug_visualize(x_path, y_path, z_path, blinker, x_new, y_new, z_new, blinker_new, distances, new_distances):
    # plot 3 figures
    # 1. x-y path with old and new waypoints
    # 2. z path with old and new waypoints
    # 3. blinker path with old and new waypoints

    fig = plt.figure(figsize=(10, 15))
    ax = fig.subplots()
    ax.scatter(x_path,y_path, color = 'blue')
    ax.scatter(x_new, y_new, color = 'red', marker = 'x', alpha = 0.5, label = 'interpolated')
    plt.legend()
    plt.show()

    # new plot for heights 
    fig = plt.figure(figsize=(10, 15))
    ax = fig.subplots()
    ax.scatter(new_distances, z_new, color = 'red', marker = 'x', alpha = 0.5, label = 'height interpolated')
    ax.plot(new_distances, z_new, color = 'red', alpha = 0.5, label = 'height interpolated')
    ax.scatter(distances, z_path, color = 'blue', alpha = 0.5, label = 'height old')
    plt.legend()
    plt.show()

    # new plot for blinkers
    fig = plt.figure(figsize=(10, 15))
    ax = fig.subplots()
    ax.scatter(new_distances, blinker_new, color = 'red', marker = 'x', alpha = 0.5, label = 'blinker interpolated')
    ax.plot(new_distances, blinker_new, color = 'red', alpha = 0.5, label = 'blinker interpolated')
    ax.scatter(distances, blinker, color = 'blue', alpha = 0.5, label = 'blinker old')
    plt.legend()
    plt.show()


if __name__ == '__main__':
    rospy.init_node('local_planner')
    node = LocalPlanner()
    node.run()