// bridge_rawsensors.cpp
#include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"
#include "ros1_bridge/bridge.hpp"

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "bridge_rawsensor_ros1_to_ros2");
  ros::NodeHandle ros1_node;

  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("bridge_rawsensor_ros1_to_ros2");

  try {
    auto bridge = ros1_bridge::create_bridge_from_1_to_2(
      ros1_node,
      ros2_node,
      "advanced_navigation_driver/RawSensorsPacket",
      "/n101xw/ins/RawSensors",
      10,
      "advanced_navigation_driver/msg/RawSensorsPacket",
      "/n101xw/ins/RawSensors",
      10);

    RCLCPP_INFO(ros2_node->get_logger(), "Bridged /n101xw/ins/RawSensors from ROS 1 to ROS 2");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    rclcpp::executors::SingleThreadedExecutor executor;
    while (ros1_node.ok() && rclcpp::ok()) {
      executor.spin_node_once(ros2_node, std::chrono::milliseconds(100));
    }

  } catch (const std::exception & e) {
    RCLCPP_ERROR(ros2_node->get_logger(), "Failed to bridge RawSensors: %s", e.what());
    return 1;
  }

  return 0;
}
