#include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"
#include "ros1_bridge/bridge.hpp"

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "selected_topics_static_bridge");
  ros::NodeHandle ros1_node;

  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("selected_topics_static_bridge");

  std::vector<ros1_bridge::Bridge1to2Handles> bridges_1to2;
  std::vector<ros1_bridge::Bridge2to1Handles> bridges_2to1;

  try {
    // ROS1 -> ROS2
    bridges_1to2.push_back(ros1_bridge::create_bridge_from_1_to_2(
      ros1_node, ros2_node,
      "std_msgs/Header", "/GpsOutageStart", 10,
      "std_msgs/msg/Header", "/GpsOutageStart", 10));
    RCLCPP_INFO(ros2_node->get_logger(), "Bridged /GpsOutageStart (ROS1 -> ROS2)");

    bridges_1to2.push_back(ros1_bridge::create_bridge_from_1_to_2(
      ros1_node, ros2_node,
      "sensor_msgs/Image", "/n101xw/cameras/eo_runway/viewer/image", 10,
      "sensor_msgs/msg/Image", "/n101xw/cameras/eo_runway/viewer/image", 10));
    RCLCPP_INFO(ros2_node->get_logger(), "Bridged /n101xw/cameras/eo_runway/viewer/image (ROS1 -> ROS2)");

    bridges_1to2.push_back(ros1_bridge::create_bridge_from_1_to_2(
      ros1_node, ros2_node,
      "", "/n101xw/ins/NavState", 10,
      "superpilot_interfaces/msg/NavState", "/n101xw/ins/NavState", 10));
    RCLCPP_INFO(ros2_node->get_logger(), "Bridged /n101xw/ins/NavState (ROS1 -> ROS2)");

    bridges_1to2.push_back(ros1_bridge::create_bridge_from_1_to_2(
      ros1_node, ros2_node,
      "advanced_navigation_driver/RawGNSSPacket", "/n101xw/ins/RawGNSS", 10,
      "advanced_navigation_driver/msg/RawGNSSPacket", "/n101xw/ins/RawGNSS", 10));
    RCLCPP_INFO(ros2_node->get_logger(), "Bridged /n101xw/ins/RawGNSS (ROS1 -> ROS2)");

    bridges_1to2.push_back(ros1_bridge::create_bridge_from_1_to_2(
      ros1_node, ros2_node,
      "advanced_navigation_driver/SystemStatePacket", "/n101xw/ins/SystemState", 10,
      "advanced_navigation_driver/msg/SystemStatePacket", "/n101xw/ins/SystemState", 10));
    RCLCPP_INFO(ros2_node->get_logger(), "Bridged /n101xw/ins/SystemState (ROS1 -> ROS2)");

    // ROS2 -> ROS1
    bridges_2to1.push_back(ros1_bridge::create_bridge_from_2_to_1(
      ros2_node, ros1_node,
      "advanced_navigation_driver/msg/SystemStatePacket", "/n101xw/nav_filter/SystemState", 10,
      "advanced_navigation_driver/SystemStatePacket", "/n101xw/nav_filter/SystemState", 10));
    RCLCPP_INFO(ros2_node->get_logger(), "Bridged /n101xw/nav_filter/SystemState (ROS2 -> ROS1)");

    bridges_2to1.push_back(ros1_bridge::create_bridge_from_2_to_1(
      ros2_node, ros1_node,
      "superpilot_interfaces/msg/VisionBasedLanding", "/n101xw/vision_based_landing/output", 10,
      "", "/n101xw/vision_based_landing/output", 10));
    RCLCPP_INFO(ros2_node->get_logger(), "Bridged /n101xw/vision_based_landing/output (ROS2 -> ROS1)");

  } catch (const std::exception & e) {
    RCLCPP_ERROR(ros2_node->get_logger(), "Bridge creation failed: %s", e.what());
    return 1;
  }

  ros::AsyncSpinner spinner(1);
  spinner.start();

  rclcpp::executors::SingleThreadedExecutor executor;
  while (ros1_node.ok() && rclcpp::ok()) {
    executor.spin_node_once(ros2_node, std::chrono::milliseconds(100));
  }

  return 0;
}
