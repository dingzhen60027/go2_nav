#include "fast_icp_loc/fast_icp_loc.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fast_icp_loc::FastIcpLoc>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
