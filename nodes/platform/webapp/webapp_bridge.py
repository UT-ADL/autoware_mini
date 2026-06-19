#!/usr/bin/env python3

import re
import rospy
import os
import dotenv
import json
import csv
import traceback
import rospkg
import secrets
import lanelet2
import paho.mqtt.client as mqtt

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.srv import GetPlan
from autoware_mini.msg import Path, Log
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import Bool

QOS_AT_MOST_ONCE = 0
QOS_AT_LEAST_ONCE = 1
QOS_EXACTLY_ONCE = 2

class WebappBridge:
    def __init__(self):
        
        # Parameters
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")
        
        self.mqtt_host = rospy.get_param("~host")
        self.mqtt_port = rospy.get_param("~port")
        self.mqtt_tls_enabled = rospy.get_param("~tls_enabled")
        self.public_url = rospy.get_param("~website_public_url")
        
        # Generate a random 6-digit session id, if connecting first time
        # Duplicate session id check is currently not implemented
        self.session_id = rospy.get_param("~session_id")
        if self.session_id is None or self.session_id == "":
            self.session_id = f"{secrets.randbelow(1_000_000):06d}"
        else:
            self.session_id = f"{int(self.session_id):06d}"

        # MQTT published topics
        self.mqtt_topic_published_status = f"session/{self.session_id}/status"
        self.mqtt_topic_published_preview_route = f"session/{self.session_id}/preview_route"

        # MQTT received topics
        self.mqtt_topic_received_goal = f"session/{self.session_id}/goal"
        self.mqtt_topic_received_rating = f"session/{self.session_id}/rating"
        self.mqtt_topic_received_preview_route = f"session/{self.session_id}/preview_route_request"
        
        # Load MQTT secrets
        # Load .env from the root directory of the repo
        dotenv_path = os.path.join(os.path.dirname(__file__), "../../../.env")
        dotenv.load_dotenv(dotenv_path)
        self.mqtt_username = os.getenv("MQTT_USERNAME")
        self.mqtt_password = os.getenv("MQTT_PASSWORD")
        
        # Kill ROS if webapp MQTT username or password is not set
        if self.mqtt_username is None or self.mqtt_password is None:
            rospy.logerr("Webapp MQTT username or password is not set! Either .env file is missing or faulty.")
            rospy.signal_shutdown("Webapp MQTT username or password is not set!")
        
        # Initialize MQTT client
        self.client = mqtt.Client(transport="websockets")
        self.client.ws_set_options(path="/")
        self.client.username_pw_set(self.mqtt_username, self.mqtt_password)
        self.client.on_connect = self.on_mqtt_connect
        self.client.on_message = self.on_mqtt_message
        
        if self.mqtt_tls_enabled:
            # Setup TLS (this uses the system's default CA store)
            self.client.tls_set()
        
        # Other initializations
        origin = lanelet2.io.Origin(utm_origin_lat, utm_origin_lon)
        self.projector = lanelet2.projection.UtmProjector(origin, use_custom_origin, False)
        self.current_pose = None
        self.current_velocity = None
        self.current_route = []

        # Can be one of: 
        # 'waiting'
        # 'driving-to-pickup', 
        # 'arrived-to-pickup', 
        # 'driving-to-destination', 
        # 'arrived-to-destination', 
        self.current_state = 'waiting'
        self.chosen_spots = {}
        self.chosen_route_stops = {}

        # Can be one of:
        # 'pickup'
        # 'dropoff'
        # 'done'
        self.goal_type = 'done'

        # Vehicle status fields for webapp
        self.drivemode_status = ""
        self.planner_status = ""
        self.assistance_active = False

        # Publishers (publish to ROS, subscribe to MQTT)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.next_stop_pub = rospy.Publisher('/dashboard/webapp_next_stop', OverlayText, queue_size=1, latch=True)

        # Subscribers (subscribe to ROS, publish to MQTT)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/planning/global_path', Path, self.global_path_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/dashboard/vehicle_drivemode', OverlayText, self.vehicle_drivemode_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/dashboard/log_message', Log, self.log_message_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/planning/assistance_enabled', Bool, self.assistance_state_callback, queue_size=1, tcp_nodelay=True)

        # Service proxy for route preview
        self.get_plan_service = rospy.ServiceProxy('/planning/get_plan', GetPlan)

        # Connect to the MQTT host
        self.client.connect(self.mqtt_host, self.mqtt_port)

        # Publish MQTT topics with a fixed frequency
        rospy.Timer(rospy.Duration(1.0), self.publish_mqtt_topics)

        # Log the WebApp URL with node name
        rospy.loginfo(f"{rospy.get_name()} - initialized\n\nWebApp URL: {self.public_url}?session_id={self.session_id}\n")
        # Publish the Session ID to the Rviz as well
        rospy.Publisher('/dashboard/webapp_session_id', OverlayText, queue_size=1, latch=True).publish(
            OverlayText(text=f"Webapp: {self.session_id}")
        )

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            # Subscribe to desired MQTT topics after successful connection
            self.client.subscribe(self.mqtt_topic_received_goal, qos=QOS_AT_LEAST_ONCE)
            self.client.subscribe(self.mqtt_topic_received_rating, qos=QOS_AT_LEAST_ONCE)
            self.client.subscribe(self.mqtt_topic_received_preview_route, qos=QOS_AT_LEAST_ONCE)
        else:
            rospy.logerr(f"Failed to connect to MQTT! Reason: {mqtt.connack_string(rc)}")
        
    def on_mqtt_message(self, client, userdata, msg):
        # General handler for receiving mqtt messages
        data = json.loads(msg.payload.decode())
        if msg.topic == self.mqtt_topic_received_goal:
            self.on_goal_point_receive(data)
        elif msg.topic == self.mqtt_topic_received_rating:
            self.on_rating_receive(data)
        elif msg.topic == self.mqtt_topic_received_preview_route:
            self.on_preview_route_receive(data)

    def on_goal_point_receive(self, data):
        # Can be one of:
        # 'pickup'
        # 'dropoff'
        # 'done'
        self.goal_type = data["type"]
        self.chosen_spots = data["chosen_spots"]
        self.chosen_route_stops = data["chosen_route_stops"]
        position = data["position"]

        self.publish_next_stop_overlay()

        # If we are done, change the car state and immediately publish
        if self.goal_type == "done":
            self.current_state = "waiting"
            return  # No further processing if we are done

        # Extract pose attributes
        latitude = position["lat"]
        longitude = position["lng"]
        height = position["height"]

        # convert pose x and pose y to local coords
        gps_point = lanelet2.core.GPSPoint(latitude, longitude, height)
        utm_point = self.projector.forward(gps_point)

        # Create a PoseStamped message
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = utm_point.x
        goal_msg.pose.position.y = utm_point.y
        goal_msg.pose.position.z = utm_point.z
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0

        # Publish the goal
        self.goal_pub.publish(goal_msg)
        
    def publish_next_stop_overlay(self):
        """Show the next bus stop while driving; clear otherwise."""
        stop = None
        if self.current_state.startswith("driving-to-") and self.chosen_route_stops:
            stop = self.chosen_route_stops.get(self.goal_type)
        if not stop:
            self.next_stop_pub.publish(OverlayText(text=""))
            return

        name = stop.get("name", "")
        district = stop.get("district", "")
        html = f"<div style='color: rgb(242, 223, 46);'>{name}</div>"
        if district:
            html += f"<div style='color: rgb(204, 188, 35);'>{district}</div>"
        self.next_stop_pub.publish(OverlayText(text=html))

    def on_rating_receive(self, data):
        datetime = data["datetime"]
        rating = data["rating"]
        csv_path = os.path.join(rospkg.RosPack().get_path('autoware_mini'), "data/webapp/ratings.csv")
        with open(csv_path, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([datetime, rating])

    def on_preview_route_receive(self, data):
        start = data["start"]
        end = data["end"]

        # Convert lat/lon to UTM coordinates
        start_gps_point = lanelet2.core.GPSPoint(start["lat"], start["lng"], start["height"])
        start_utm_point = self.projector.forward(start_gps_point)

        end_gps_point = lanelet2.core.GPSPoint(end["lat"], end["lng"], end["height"])
        end_utm_point = self.projector.forward(end_gps_point)

        # Build GetPlan request
        start_pose = PoseStamped()
        start_pose.header.frame_id = "map"
        start_pose.header.stamp = rospy.Time.now()
        start_pose.pose.position.x = start_utm_point.x
        start_pose.pose.position.y = start_utm_point.y
        start_pose.pose.position.z = start_utm_point.z
        start_pose.pose.orientation.w = 1.0

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.pose.position.x = end_utm_point.x
        goal_pose.pose.position.y = end_utm_point.y
        goal_pose.pose.position.z = end_utm_point.z
        goal_pose.pose.orientation.w = 1.0

        try:
            response = self.get_plan_service(start_pose, goal_pose, 0.0)
        except rospy.ServiceException as e:
            rospy.logerr("%s - get_plan service call failed: %s", rospy.get_name(), e)
            result = {"route": [], "success": False}
            self.client.publish(self.mqtt_topic_published_preview_route, json.dumps(result))
            return

        route_list = []
        for pose_stamped in response.plan.poses:
            pos = pose_stamped.pose.position
            utm_point = lanelet2.core.BasicPoint3d(pos.x, pos.y, pos.z)
            gps_point = self.projector.reverse(utm_point)
            # Speed is encoded in orientation.x by the planner (m/s -> km/h)
            speed_kmh = round(pose_stamped.pose.orientation.x * 3.6, 2)
            route_list.append({"lat": gps_point.lat, "lng": gps_point.lon, "height": gps_point.ele, "speed_kmh": speed_kmh})

        success = len(route_list) > 0
        result = {"route": route_list, "success": success}
        self.client.publish(self.mqtt_topic_published_preview_route, json.dumps(result))
    
    def publish_mqtt_topics(self, timer_event):
        try:
            if self.current_velocity is not None and self.current_pose is not None:
                pos = self.current_pose.pose.position
                utm_point = lanelet2.core.BasicPoint3d(pos.x, pos.y, pos.z)
                gps_point = self.projector.reverse(utm_point)
                speed = round(self.current_velocity.twist.linear.x * 3.6, 2)  # Convert m/s to km/h and round to 2 decimal places
                data = {
                    "pose": {
                        "lat": gps_point.lat,
                        "lng": gps_point.lon,
                        "height": gps_point.ele
                    },
                    "speed": speed,
                    "state": self.current_state,
                    "route": self.current_route,
                    "chosen_spots": self.chosen_spots,
                    "chosen_route_stops": self.chosen_route_stops,
                    "drivemode_status": self.drivemode_status,
                    "planner_status": self.planner_status,
                }
                self.client.publish(self.mqtt_topic_published_status, json.dumps(data))
        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in publish_mqtt_topics: %s", rospy.get_name(), traceback.format_exc())

    def current_pose_callback(self, msg):
        """Overwrite current pose."""
        self.current_pose = msg
    
    def current_velocity_callback(self, msg):
        """Overwrite current velocity."""
        self.current_velocity = msg

    def vehicle_drivemode_callback(self, msg):
        """Update drive mode status from dashboard overlay text, stripping HTML tags."""
        self.drivemode_status = re.sub(r'<[^>]+>', '', msg.text).strip()

    def log_message_callback(self, msg):
        """Update planner status from dashboard log."""
        if msg.instant == False:
            self.planner_status = msg.message

    def assistance_state_callback(self, msg):
        """Track whether remote assistance is currently active."""
        self.assistance_active = bool(msg.data)

    def global_path_callback(self, msg):
        """Publish the waypoints list from the global path to MQTT."""

        # During remote assistance, keep the last known route and state.
        # The assisted path is temporary and should not affect the app.
        if self.assistance_active:
            return

        # Convert list of waypoints to list of latlon coordinates with speed
        route = []
        for w in msg.waypoints:
            utm_point = lanelet2.core.BasicPoint3d(w.position.x, w.position.y, w.position.z)
            gps_point = self.projector.reverse(utm_point)
            speed_kmh = round(w.speed * 3.6, 2)  # Convert m/s to km/h
            route.append((gps_point.lat, gps_point.lon, gps_point.ele, speed_kmh))

        self.current_route = route

        if self.goal_type == "pickup":
            if self.current_route:
                self.current_state = "driving-to-pickup"
            else:
                self.current_state = "arrived-to-pickup"
        elif self.goal_type == "dropoff":
            if self.current_route:
                self.current_state = "driving-to-destination"
            else:
                self.current_state = "arrived-to-destination"

        self.publish_next_stop_overlay()

    def run(self):
        # Start the MQTT client loop in a separate thread
        self.client.loop_start()

        # Spin the ROS node
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('webapp_bridge')
    node = WebappBridge()
    node.run()
    