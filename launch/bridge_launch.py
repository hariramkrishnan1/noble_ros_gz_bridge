import os
import subprocess
import shutil

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def auto_install_ros_gz_bridge():
    try:
        if shutil.which("ros2") is None:
            raise RuntimeError("ROS 2 is not installed or not sourced properly.")

        result = subprocess.run(["ros2", "pkg", "list"], stdout=subprocess.PIPE, check=True)
        if b"ros_gz_bridge" in result.stdout:
            print("[INFO] ros_gz_bridge is already installed.")
        else:
            print("[INFO] Installing ros_gz_bridge...")
            subprocess.check_call(["sudo", "apt", "update"])
            subprocess.check_call(["sudo", "apt", "install", "-y", "ros-jazzy-ros-gz-bridge"])
            print("[INFO] ros_gz_bridge installed successfully.")
    except Exception as e:
        print(f"[ERROR] Failed to auto-install ros_gz_bridge: {e}")

def generate_launch_description():
    auto_install_ros_gz_bridge()

    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(
                os.path.dirname(__file__),
                'bridge.yaml'
            ),
            description='YAML configuration file for bridge.'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_parameter_bridge',
            output='screen',
            parameters=[config_file],
        )
    ])