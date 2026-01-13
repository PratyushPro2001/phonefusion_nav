#include <rclcpp/rclcpp.hpp>
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("phone_cam_bridge");
  RCLCPP_INFO(node->get_logger(), "phone_cam_bridge stub running.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
