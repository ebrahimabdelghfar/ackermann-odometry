#include "ackermann_odometry/odom_publisher.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ackermann_odometry::OdomPublisher>());
  rclcpp::shutdown();
  return 0;
}
