#ifndef ACKERMANN_ODOMETRY__ODOM_PUBLISHER_HPP_
#define ACKERMANN_ODOMETRY__ODOM_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_interfaces/msg/ackermann_feedback.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <limits>

namespace ackermann_odometry
{

struct AckermannState
{
  double x;
  double y;
  double z;
  tf2::Quaternion orientation;
  double left_wheel_speed;
  double right_wheel_speed;
  double steering_angle;
  rclcpp::Time time;

  AckermannState()
  : x(0.0), y(0.0), z(0.0),
    orientation(0.0, 0.0, 0.0, 1.0),
    left_wheel_speed(0.0),
    right_wheel_speed(0.0),
    steering_angle(0.0)
  {
  }
};

class OdomPublisher : public rclcpp::Node
{
public:
  OdomPublisher();

private:
  void feedback_callback(const ackermann_interfaces::msg::AckermannFeedback::SharedPtr msg);
  AckermannState state_update(const AckermannState & state, 
                               const ackermann_interfaces::msg::AckermannFeedback::SharedPtr & feedback);
  nav_msgs::msg::Odometry output(const AckermannState & state);
  double turn_radius(double steering_angle);
  void linear_velocity(const tf2::Quaternion & orientation, double speed, 
                       double & vx, double & vy, double & vz);

  // Parameters
  double axle_length_;
  double wheelbase_length_;
  double wheel_radius_;
  double center_of_mass_offset_;
  double damping_factor_;

  // State
  AckermannState state_;

  // Publishers and Subscribers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<ackermann_interfaces::msg::AckermannFeedback>::SharedPtr subscription_;
};

}  // namespace ackermann_odometry

#endif  // ACKERMANN_ODOMETRY__ODOM_PUBLISHER_HPP_
