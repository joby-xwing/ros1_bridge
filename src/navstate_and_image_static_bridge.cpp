#include <string>
#include <vector>

#include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"
#include "ros1_bridge/bridge.hpp"

int main(int argc, char * argv[])
{
  // ROS 1 initialization
  ros::init(argc, argv, "bridge_ros1_to_ros2");
  ros::NodeHandle ros1_node;

  // ROS 2 initialization
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("bridge_ros1_to_ros2");

  std::vector<ros1_bridge::Bridge1to2Handles> bridges;

  try {
    // Bridge for /n101xw/ins/NavState
    auto bridge_navstate = ros1_bridge::create_bridge_from_1_to_2(
      ros1_node,
      ros2_node,
      "superpilot_interfaces/msg/NavState",     // ROS 1 type name
      "/n101xw/ins/NavState",                   // ROS 1 topic name
      10,                                       // ROS 1 subscriber queue
      "superpilot_interfaces/msg/NavState",     // ROS 2 type name
      "/n101xw/ins/NavState",                   // ROS 2 topic name
      10);                                      // ROS 2 publisher queue

    bridges.push_back(bridge_navstate);
    RCLCPP_INFO(ros2_node->get_logger(), "Bridged /n101xw/ins/NavState from ROS 1 to ROS 2");

    // Bridge for /n101xw/cameras/eo_runway/viewer/image
    auto bridge_image = ros1_bridge::create_bridge_from_1_to_2(
      ros1_node,
      ros2_node,
      "sensor_msgs/Image",
      "/n101xw/cameras/eo_runway/viewer/image",
      10,
      "sensor_msgs/msg/Image",
      "/n101xw/cameras/eo_runway/viewer/image",
      10);

    bridges.push_back(bridge_image);
    RCLCPP_INFO(ros2_node->get_logger(), "Bridged /n101xw/cameras/eo_runway/viewer/image from ROS 1 to ROS 2");

  } catch (const std::exception & e) {
    RCLCPP_ERROR(ros2_node->get_logger(), "Bridge creation failed: %s", e.what());
    return 1;
  }

  // Start spinning
  ros::AsyncSpinner spinner(1);
  spinner.start();

  rclcpp::executors::SingleThreadedExecutor executor;
  while (ros1_node.ok() && rclcpp::ok()) {
    executor.spin_node_once(ros2_node, std::chrono::milliseconds(100));
  }

  return 0;
}
