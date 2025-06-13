#!/usr/bin/env python3

import rospy, os, dotenv, json, csv, threading, secrets
import paho.mqtt.client as mqtt

from autoware_mini.localization import WGS84ToUTMTransformer
from geometry_msgs.msg import PoseStamped, TwistStamped
from autoware_mini.msg import Path


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
        
        self.session_id = rospy.get_param("~session_id", None)
        
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
        self.client.ws_set_options(path="")
        self.client.username_pw_set(self.mqtt_username, self.mqtt_password)
        self.client.on_connect = self.on_mqtt_connect
        self.client.on_message = self.on_mqtt_message
        
        if self.mqtt_tls_enabled:
            # Setup TLS (this uses the system's default CA store)
            self.client.tls_set()
        
        # Other initializations
        self.converter = WGS84ToUTMTransformer(use_custom_origin, utm_origin_lat, utm_origin_lon)
        self.current_pose = None
        self.current_velocity = None
        
        # MQTT published topics
        self.mqtt_topic_published_status = None
        self.mqtt_topic_published_route = None
        
        # MQTT received topics
        self.mqtt_topic_received_goal = None
        self.mqtt_topic_received_rating = None
        
        # Event to block until the initial webapp mqtt connection is established
        self.connection_event = threading.Event()
                    
        # Publishers (publish to ROS, subscribe to MQTT)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, tcp_nodelay=True)
        
        # Subscribers (subscribe to ROS, publish to MQTT)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/planning/global_path', Path, self.global_path_callback, queue_size=1, tcp_nodelay=True)
        
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            rospy.loginfo(f"Connected to MQTT!")
            
            # Generate a random 6-digit session id, if connecting first time
            # Duplicate session id check is currently not implemented
            if not self.session_id:
                self.session_id = f"{secrets.randbelow(1_000_000):06d}"
            
            rospy.loginfo(f"WebApp Public URL: {self.public_url}?session_id={self.session_id}")
            
            # MQTT published topics
            self.mqtt_topic_published_status = f"session/{self.session_id}/status"
            self.mqtt_topic_published_route = f"session/{self.session_id}/route"
            
            # MQTT received topics
            self.mqtt_topic_received_goal = f"session/{self.session_id}/goal"
            self.mqtt_topic_received_rating = f"session/{self.session_id}/rating"
            
            # Subscribe to desired MQTT topics after successful connection
            self.client.subscribe(self.mqtt_topic_received_goal, qos=1)
            self.client.subscribe(self.mqtt_topic_received_rating, qos=1)
            
            self.connection_event.set()  # Notify that the connection is established
        else:
            rospy.logerr(f"Failed to connect to MQTT! Reason: {mqtt.connack_string(rc)}")
        
    def on_mqtt_message(self, client, userdata, msg):
        # General handler for receiving mqtt messages
        data = json.loads(msg.payload.decode())
        if msg.topic == self.mqtt_topic_received_goal:
            self.on_goal_point_receive(data)
        elif msg.topic == self.mqtt_topic_received_rating:
            self.on_rating_receive(data)
    
    def on_goal_point_receive(self, data):
        # Extract pose attributes
        latitude = data["lat"]
        longitude = data["lng"]
        height = data["height"]
        
        # convert pose x and pose y to local coords
        pose_x, pose_y = self.converter.transform_lat_lon(latitude, longitude, height)

        # Create a PoseStamped message
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = pose_x
        goal_msg.pose.position.y = pose_y
        goal_msg.pose.position.z = height
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0

        # Publish the goal
        self.goal_pub.publish(goal_msg)
        
    def on_rating_receive(self, data):
        datetime = data["datetime"]
        rating = data["rating"]
        csv_path = os.path.join(os.path.dirname(__file__), "../../../data/webapp/ratings.csv")
        with open(csv_path, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([datetime, rating])
    
    def publish_throttled_mqtt_topics(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.current_velocity is not None and self.current_pose is not None:
                
                lat, lon = self.converter.transform_utm(self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z)
                speed = round(self.current_velocity.twist.linear.x * 3.6, 2)  # Convert m/s to km/h and round to 2 decimal places
                data = {
                    "pose": {
                        "lat": lat,
                        "lng": lon   
                    },
                    "speed": speed
                }
                self.client.publish(self.mqtt_topic_published_status, json.dumps(data))
            rate.sleep()
    
    def current_pose_callback(self, msg):
        """Overwrite current pose."""
        self.current_pose = msg
    
    def current_velocity_callback(self, msg):
        """Overwrite current velocity."""
        self.current_velocity = msg
    
    def global_path_callback(self, msg):
        """Publish the waypoints list from the global path to MQTT."""
        
        # Convert list of waypoints to list of latlon coordinates
        wp_latlons = [self.converter.transform_utm(w.position.x, w.position.y, w.position.z) for w in msg.waypoints]
        data = {
            "waypoints": wp_latlons
        }
        self.client.publish(self.mqtt_topic_published_route, json.dumps(data),  qos=1)
    
    def run(self):
        # Connect to the MQTT host
        self.client.connect(self.mqtt_host, self.mqtt_port)
        
        # Start the MQTT client loop in a separate thread
        self.client.loop_start()
        
        # Block until the first connection is established
        rospy.loginfo("Waiting for MQTT connection...")
        self.connection_event.wait()  # Blocks until `self.connection_event.set()` is called
        rospy.loginfo("Proceeding with the webapp node.")
        
        # Spin the ROS node in a separate thread
        threading.Thread(target=rospy.spin, daemon=True).start()
        
        # Start publishing data to MQTT periodically
        self.publish_throttled_mqtt_topics()

if __name__ == '__main__':
    rospy.init_node('webapp_bridge')
    node = WebappBridge()
    node.run()
    