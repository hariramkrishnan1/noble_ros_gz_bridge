#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
import os
from ros_gz_bridge.bridge import BridgeFactory

class DynamicBridgeNode(Node):
    def __init__(self, config_file):
        super().__init__('dynamic_bridge_node')
        self.get_logger().info(f"Loading bridge config from: {config_file}")
        self.bridge_factory = BridgeFactory(node=self)
        self._load_config(config_file)

    def _load_config(self, path):
        with open(path, 'r') as f:
            config = yaml.safe_load(f)

        # Topics
        for topic in config.get("bridge", {}).get("topics", []):
            direction = topic["direction"]
            if direction == "gz_to_ros":
                self.bridge_factory.gz_to_ros(topic["gz_type"], topic["ros_type"], topic["topic_name"])
            elif direction == "ros_to_gz":
                self.bridge_factory.ros_to_gz(topic["ros_type"], topic["gz_type"], topic["topic_name"])
            elif direction == "bidirectional":
                self.bridge_factory.gz_to_ros(topic["gz_type"], topic["ros_type"], topic["topic_name"])
                self.bridge_factory.ros_to_gz(topic["ros_type"], topic["gz_type"], topic["topic_name"])

        # Services
        for service in config.get("bridge", {}).get("services", []):
            direction = service["direction"]
            if direction == "ros_to_gz":
                self.bridge_factory.ros_service_to_gz(service["ros_type"], service["gz_type"], service["service_name"])
            elif direction == "gz_to_ros":
                self.bridge_factory.gz_service_to_ros(service["gz_type"], service["ros_type"], service["service_name"])

        # Actions
        for action in config.get("bridge", {}).get("actions", []):
            direction = action["direction"]
            if direction == "ros_to_gz":
                self.bridge_factory.ros_action_to_gz(action["ros_type"], action["gz_type"], action["action_name"])
            elif direction == "gz_to_ros":
                self.bridge_factory.gz_action_to_ros(action["gz_type"], action["ros_type"], action["action_name"])

def main():
    import sys
    rclpy.init(args=sys.argv)
    config_path = sys.argv[1] if len(sys.argv) > 1 else "config/bridge.yaml"
    node = DynamicBridgeNode(config_path)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
