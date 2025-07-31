#include <llcp_ros.h>

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<MrsLlcpRos>(rclcpp::NodeOptions());
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}