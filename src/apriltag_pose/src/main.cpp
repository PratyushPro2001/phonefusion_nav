#include <rclcpp/rclcpp.hpp>
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("apriltag_pose");
  RCLCPP_INFO(node->get_logger(), "apriltag_pose stub running.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
