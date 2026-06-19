#!/usr/bin/env python3

import argparse
import sys
import rospy
from std_srvs.srv import Empty
from autoware_mini.msg import StopLineStatusArray, StopLineStatus
import StreamDeck.DeviceManager
import StreamDeck.ImageHelpers.PILHelper


# Button states mapping for StreamDeck colors
STATE_COLORS = {
    "ready": "gray",
    "stop": "red",
    "go": "lime",
}

CLIENT_STATES = {
    StopLineStatus.STATUS_UNKNOWN: "ready",
    StopLineStatus.STATUS_STOP: "stop",
    StopLineStatus.STATUS_GO: "go",
}

class ButtonController:
    def __init__(self, verbose=False):
        # StreamDeck parameters
        self.deck = None
        self.key_index = 0  # Top-left button

        # Other parameters
        self.verbose = verbose
        self.state = "ready"

        if self.verbose:
            print("[INFO] StreamDeck init start")
        self._init_streamdeck()
        if self.verbose:
            print("[INFO] StreamDeck init success")

        # Prerender images for all states after deck is initialized
        self.state_images = {}
        for state, color in STATE_COLORS.items():
            self.state_images[state] = self.render_key_image(color)
        self.update_key_image()  # Initialize the key image

        # Ensure StreamDeck is cleaned up on ROS shutdown
        rospy.on_shutdown(self.shutdown)

        # ROS service initialization
        self.service_confirm_drive = rospy.ServiceProxy("/planning/service_confirm_drive", Empty)

        # Subscribe to stop line statuses
        rospy.Subscriber(
            "/planning/stop_line_status",
            StopLineStatusArray,
            self.stop_line_status_callback,
            queue_size=1,
            tcp_nodelay=True,
        )

    def _init_streamdeck(self):
        streamdecks = StreamDeck.DeviceManager.DeviceManager().enumerate()
        if not streamdecks:
            print("No Stream Deck found.", file=sys.stderr)
            sys.exit(1)
        self.deck = streamdecks[0]
        if not self.deck.is_visual():
            print("Stream Deck is not visual.", file=sys.stderr)
            sys.exit(1)
        self.deck.open()
        self.deck.reset()
        self.deck.set_brightness(100)
        self.deck.set_key_callback(self.key_change_callback)

    def render_key_image(self, color):
        image = StreamDeck.ImageHelpers.PILHelper.create_key_image(self.deck, color)
        return StreamDeck.ImageHelpers.PILHelper.to_native_key_format(self.deck, image)

    def update_key_image(self):
        with self.deck:
            self.deck.set_key_image(self.key_index, self.state_images[self.state])

    def key_change_callback(self, _deck, key, state):
        if key != self.key_index:
            return
        if state:  # Button pressed
            if self.verbose:
                print(f"[INFO] Button pressed (state={self.state})")
            # If operator presses while vehicle is stopped, call confirm_drive service
            if self.state == "stop":
                if self.service_confirm_drive is None:
                    if self.verbose:
                        print("[WARN] service_confirm_drive not ready")
                    return
                try:
                    if self.verbose:
                        print("[INFO] Calling service_confirm_drive service")
                    self.service_confirm_drive()
                except Exception as e:
                    rospy.logerr(f"Failed to call service_confirm_drive service: {e}")

    def stop_line_status_callback(self, msg: StopLineStatusArray):
        # use YIELD_MANUAL statuses from StopLineStatusArray
        if msg.type == StopLineStatusArray.YIELD_MANUAL:
            stop_lines = msg.statuses
            if len(stop_lines) > 0:
                closest_stop_line = stop_lines[0]
                closest_stop_line_status = closest_stop_line.status
            else:
                closest_stop_line_status = StopLineStatus.STATUS_UNKNOWN

            # Update StreamDeck state based on the closest stop line status
            new_state = CLIENT_STATES[closest_stop_line_status]
            if new_state != self.state:
                if self.verbose:
                    print(f"[INFO] Button state changed from '{self.state}' to '{new_state}'")
                self.state = new_state
                self.update_key_image()

    def run(self):
        rospy.spin()

    def shutdown(self):
        if self.verbose:
            print("[INFO] Shutting down StreamDeck controller...")
        with self.deck:
            self.deck.reset()
            self.deck.close()
        if self.verbose:
            print("[INFO] StreamDeck controller shut down.")

def parse_args():
    parser = argparse.ArgumentParser(description="Remote Assistance Button Controller")
    parser.add_argument("--verbose", action="store_true", help="Enable verbose logging")
    return parser.parse_args()

if __name__ == "__main__":
    args = parse_args()

    rospy.init_node("button_controller")
    controller = ButtonController(verbose=args.verbose)
    controller.run()
