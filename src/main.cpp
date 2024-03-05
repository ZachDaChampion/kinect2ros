#include "kinect2ros/kinect2ros.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  const rclcpp::NodeOptions options;
  auto node = std::make_shared<Kinect2RosNode>(options);

  // Ensure node was created without issue.
  if (!node->is_okay()) {
    RCLCPP_ERROR(node->get_logger(), "Node could not be started. Exiting...");
    return 1;
  }

  // Spin and update the node until an update fails.
  while (1) {
    rclcpp::spin_some(node);
    if (!node->update()) {
      RCLCPP_ERROR(node->get_logger(), "Node update failed. Exiting...");
      return 1;
    }
  }

  rclcpp::shutdown();
  return 0;
}