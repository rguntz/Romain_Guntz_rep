"""Simulate a keyboard press by publishing to the same ROS2 topic
KeyboardListenerPublisher normally publishes to, so a headless run can
toggle run_g1_data_exporter.py's recording state (key 'c') the same way
a real operator's keypress would.

Usage: python send_keypress.py <key>
"""
import sys
import time

import rclpy

from decoupled_wbc.control.utils.keyboard_dispatcher import KeyboardListenerPublisher

key = sys.argv[1]

rclpy.init(args=None)
node = rclpy.create_node("fake_keypress_sender")
executor = rclpy.get_global_executor()
executor.add_node(node)

publisher = KeyboardListenerPublisher()

# A freshly-created publisher isn't necessarily discovered yet by long-running
# subscriber processes (control loop, data exporter) — DDS discovery between
# separate processes isn't instant. Wait for discovery to complete, then
# publish exactly once (read_msg() consumes on read, so multiple deliveries
# received before a read would each register as a separate toggle).
time.sleep(3.0)
publisher.handle_keyboard_button(key)
time.sleep(0.5)
print(f"Sent key '{key}' (after a 3s discovery wait)")
