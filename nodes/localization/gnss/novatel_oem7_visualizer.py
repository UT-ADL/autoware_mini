#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import ColorRGBA
from novatel_oem7_msgs.msg import INSPVA, BESTPOS
from jsk_rviz_plugins.msg import OverlayText

INSPVA_STATUS = {
    0: "INS_INACTIVE",
    1: "INS_ALIGNING",
    2: "INS_HIGH_VARIANCE",
    3: "INS_SOLUTION_GOOD",
    6: "INS_SOLUTION_FREE",
    7: "INS_ALIGNMENT_COMPLETE",
    8: "DETERMINING_ORIENTATION",
    9: "WAITING_INITIALPOS",
    10: "WAITING_AZIMUTH",
    11: "INITIALIZING_BIASES",
    12: "MOTION_DETECT",
    14: "WAITING_ALIGNMENTORIENTATION"
    }

BESTPOS_POS_TYPE = {
    0: "NONE",
    1: "FIXEDPOS",
    2: "FIXEDHEIGHT",
    8: "DOPPLER_VELOCITY",
    16: "SINGLE",
    17: "PSRDIFF",
    18: "WAAS",
    19: "PROPAGATED",
    32: "L1_FLOAT",
    34: "NARROW_FLOAT",
    48: "L1_INT",
    49: "WIDE_INT",
    50: "NARROW_INT",
    51: "RTK_DIRECT_INS",
    52: "INS_SBAS",
    53: "INS_PSRSP",
    54: "INS_PSRDIFF",
    55: "INS_RTKFLOAT",
    56: "INS_RTKFIXED",
    68: "PPP_CONVERGING",
    69: "PPP",
    70: "OPERATIONAL",
    71: "WARNING",
    72: "OUT_OF_BOUNDS",
    73: "INS_PPP_CONVERGING",
    74: "INS_PPP",
    77: "PPP_BASIC_CONVERGING",
    78: "PPP_BASIC",
    79: "INS_PPP_BASIC_CONVERGING",
    80: "INS_PPP_BASIC"
    }

GREEN = ColorRGBA(0.0, 1.0, 0.0, 0.3)
YELLOW = ColorRGBA(1.0, 1.0, 0.0, 0.3)
RED = ColorRGBA(1.0, 0.0, 0.0, 0.3)

WHITE = ColorRGBA(1.0, 1.0, 1.0, 1.0)

INS_SOLUTION_GOOD = 3
INS_RTKFIXED = 56

class NovatelOem7Visualizer:
    def __init__(self):

        # Parameters
        self.number_of_satellites_good = rospy.get_param("number_of_satellites_good")
        self.number_of_satellites_bad = rospy.get_param("number_of_satellites_bad")
        self.location_accuracy_stdev_good = rospy.get_param("location_accuracy_stdev_good")
        self.location_accuracy_stdev_bad = rospy.get_param("location_accuracy_stdev_bad")
        self.differential_age_good = rospy.get_param("differential_age_good")
        self.differential_age_bad = rospy.get_param("differential_age_bad")

        # Publishers
        self.inspva_status_pub = rospy.Publisher('gnss_inspva_status', OverlayText, queue_size=1)
        self.bestpos_pos_type_pub = rospy.Publisher('gnss_bestpos_pos_type', OverlayText, queue_size=1)
        self.bestpos_num_sol_svs_pub = rospy.Publisher('gnss_bestpos_num_sol_svs', OverlayText, queue_size=1)
        self.bestpos_loc_stdev_pub = rospy.Publisher('gnss_bestpos_loc_stdev', OverlayText, queue_size=1)
        self.bestpos_diff_age_pub = rospy.Publisher('gnss_bestpos_diff_age', OverlayText, queue_size=1)

        # Subscribers
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.inspva_callback, queue_size=1)
        rospy.Subscriber('/novatel/oem7/bestpos', BESTPOS, self.bestpos_callback, queue_size=1)

        # Parameters
        self.global_top = 500
        self.global_left = 10
        self.global_width = 250
        self.global_height = 17
        self.text_size = 10


    def inspva_callback(self, msg):

        bg_color = GREEN
        if msg.status.status != INS_SOLUTION_GOOD:
            bg_color = YELLOW

        if msg.status.status not in INSPVA_STATUS:
            text = "Unknown IMU status: {}".format(msg.status.status)
        else:
            text = "IMU status: " + INSPVA_STATUS[msg.status.status]

        inspva_status = OverlayText()
        inspva_status.text = text
        inspva_status.top = self.global_top 
        inspva_status.left = self.global_left
        inspva_status.width = self.global_width
        inspva_status.height = self.global_height
        inspva_status.text_size = self.text_size
        inspva_status.fg_color = WHITE
        inspva_status.bg_color = bg_color

        self.inspva_status_pub.publish(inspva_status)

    def bestpos_callback(self, msg):
        
        ################# bestpos_pos_type
        bg_color = GREEN
        if msg.pos_type.type != INS_RTKFIXED:
            bg_color = YELLOW

        if msg.pos_type.type not in BESTPOS_POS_TYPE:
            text = "Unknown BESTPOS pos_type: {}".format(msg.pos_type.type)
        else:
            text = "GNSS position type: " + BESTPOS_POS_TYPE[msg.pos_type.type]
        
        bestpos_pos_type = OverlayText()
        bestpos_pos_type.text = text
        bestpos_pos_type.top = self.global_top + 1 * self.global_height
        bestpos_pos_type.left = self.global_left
        bestpos_pos_type.width = self.global_width
        bestpos_pos_type.height = self.global_height
        bestpos_pos_type.text_size = self.text_size
        bestpos_pos_type.fg_color = WHITE
        bestpos_pos_type.bg_color = bg_color

        self.bestpos_pos_type_pub.publish(bestpos_pos_type)

        ################# num_sol_svs
        bg_color = GREEN
        if msg.num_sol_svs < self.number_of_satellites_good:
            bg_color = YELLOW
        if msg.num_sol_svs < self.number_of_satellites_bad:
            bg_color = RED

        num_sol_svs = OverlayText()
        num_sol_svs.text = "Num. satellites: {}".format(msg.num_sol_svs)
        num_sol_svs.top = self.global_top + 2 * self.global_height
        num_sol_svs.left = self.global_left
        num_sol_svs.width = self.global_width
        num_sol_svs.height = self.global_height
        num_sol_svs.text_size = self.text_size
        num_sol_svs.fg_color = WHITE
        num_sol_svs.bg_color = bg_color

        self.bestpos_num_sol_svs_pub.publish(num_sol_svs)

        ################# loc_stdev
        location_stdev = math.sqrt(msg.lat_stdev**2 + msg.lon_stdev**2)

        bg_color = GREEN
        if location_stdev > self.location_accuracy_stdev_good:
            bg_color = YELLOW
        if location_stdev > self.location_accuracy_stdev_bad:
            bg_color = RED

        loc_stdev = OverlayText()
        loc_stdev.text = "Location stdev: {:.2f} m".format(location_stdev)
        loc_stdev.top = self.global_top + 3 * self.global_height
        loc_stdev.left = self.global_left
        loc_stdev.width = self.global_width
        loc_stdev.height = self.global_height
        loc_stdev.text_size = self.text_size
        loc_stdev.fg_color = WHITE
        loc_stdev.bg_color = bg_color

        self.bestpos_loc_stdev_pub.publish(loc_stdev)

        ################# diff_age
        bg_color = GREEN
        if msg.diff_age > self.differential_age_good:
            bg_color = YELLOW
        if msg.diff_age > self.differential_age_bad:
            bg_color = RED

        diff_age = OverlayText()
        diff_age.text = "Differential age: {:.2f} s".format(msg.diff_age)
        diff_age.top = self.global_top + 4 * self.global_height
        diff_age.left = self.global_left
        diff_age.width = self.global_width
        diff_age.height = self.global_height
        diff_age.text_size = self.text_size
        diff_age.fg_color = WHITE
        diff_age.bg_color = bg_color

        self.bestpos_diff_age_pub.publish(diff_age)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('novatel_oem7_visualizer', log_level=rospy.INFO)
    node = NovatelOem7Visualizer()
    node.run()
