#include <iostream>
#include <cstdlib>
#include <filesystem>
#include <fstream>

bool isBridgeInstalled() {
    return (std::system("ros2 pkg list | grep ros_gz_bridge > /dev/null") == 0);
}

void installBridge() {
    std::cout << "[INFO] Installing ros_gz_bridge..." << std::endl;
    std::system("sudo apt update");
    std::system("sudo apt install -y ros-jazzy-ros-gz-bridge");
}

int main(int argc, char **argv) {
    if (!isBridgeInstalled()) {
        installBridge();
    } else {
        std::cout << "[INFO] ros_gz_bridge is already installed." << std::endl;
    }

    // Launch node manually or delegate to ROS2 launch system
    std::cout << "[INFO] Launching bridge..." << std::endl;
    std::system("ros2 run ros_gz_bridge parameter_bridge --ros-args --params-file ./bridge.yaml");

    return 0;
}
