#include "ackermann_odometry/odom_publisher.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ackermann_odometry
{

OdomPublisher::OdomPublisher()
: Node("odom_publisher")
{
  // Declare and get parameters
  this->declare_parameter("axle_length", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("wheelbase_length", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("wheel_radius", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("center_of_mass_offset", 0.0);
  this->declare_parameter("damping_factor", 1.0);

  try {
    axle_length_ = this->get_parameter("axle_length").as_double();
    wheelbase_length_ = this->get_parameter("wheelbase_length").as_double();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    center_of_mass_offset_ = this->get_parameter("center_of_mass_offset").as_double();
    damping_factor_ = this->get_parameter("damping_factor").as_double();
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException & e) {
    RCLCPP_ERROR(this->get_logger(), "Not all parameters are set properly: %s", e.what());
    throw;
  }

  // Initialize state
  state_.time = this->get_clock()->now();

  // Publishers
  publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  // Subscribers
  subscription_ = this->create_subscription<ackermann_interfaces::msg::AckermannFeedback>(
    "feedback", 10,
    std::bind(&OdomPublisher::feedback_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Odometry publisher initialized");
  RCLCPP_INFO(this->get_logger(), "Parameters: axle_length=%.2f, wheelbase_length=%.2f, wheel_radius=%.2f",
              axle_length_, wheelbase_length_, wheel_radius_);
}

void OdomPublisher::feedback_callback(
  const ackermann_interfaces::msg::AckermannFeedback::SharedPtr msg)
{
  state_ = state_update(state_, msg);
  RCLCPP_DEBUG(this->get_logger(), "State updated: position=(%.3f, %.3f, %.3f)", 
               state_.x, state_.y, state_.z);
  
  auto output_msg = output(state_);
  RCLCPP_DEBUG(this->get_logger(), "Publishing odometry");
  
  publisher_->publish(output_msg);
}

AckermannState OdomPublisher::state_update(
  const AckermannState & state,
  const ackermann_interfaces::msg::AckermannFeedback::SharedPtr & feedback)
{
  // Calculate velocities
  double average_wheel_speed = (state.left_wheel_speed + state.right_wheel_speed) / 2.0;
  double linear_speed = average_wheel_speed * wheel_radius_;
  double turn_rad = turn_radius(state.steering_angle);
  double angular_speed = linear_speed / turn_rad;  // This is zero if turn_radius is infinite

  // Calculate time delta
  rclcpp::Time feedback_time(feedback->header.stamp);
  double time_delta = (feedback_time - state.time).seconds();

  // Calculate heading delta
  double heading_delta = angular_speed * time_delta;  // This is zero if angular_speed is zero

  // Calculate orientation delta
  tf2::Quaternion orientation_delta;
  orientation_delta.setRPY(0.0, 0.0, heading_delta);

  // Calculate position delta
  double position_delta_x = 0.0;
  double position_delta_y = 0.0;
  double position_delta_z = 0.0;

  if (std::isfinite(turn_rad)) {
    double lateral_delta = turn_rad * (1.0 - std::cos(heading_delta));
    double forward_delta = turn_rad * std::sin(heading_delta);
    
    // Apply orientation to relative delta
    tf2::Vector3 relative_delta(forward_delta, lateral_delta, 0.0);
    tf2::Matrix3x3 rotation_matrix(state.orientation);
    tf2::Vector3 position_delta = rotation_matrix * relative_delta;
    
    position_delta_x = position_delta.x();
    position_delta_y = position_delta.y();
    position_delta_z = position_delta.z();
  } else {
    // Straight line motion
    double vx, vy, vz;
    linear_velocity(state.orientation, linear_speed, vx, vy, vz);
    position_delta_x = time_delta * vx;
    position_delta_y = time_delta * vy;
    position_delta_z = time_delta * vz;
  }

  // Create new state
  AckermannState new_state;
  new_state.x = state.x + damping_factor_ * position_delta_x;
  new_state.y = state.y + damping_factor_ * position_delta_y;
  new_state.z = state.z + damping_factor_ * position_delta_z;
  new_state.orientation = orientation_delta * state.orientation;
  new_state.left_wheel_speed = feedback->left_wheel_speed;
  new_state.right_wheel_speed = feedback->right_wheel_speed;
  new_state.steering_angle = feedback->steering_angle;
  new_state.time = feedback_time;

  return new_state;
}

nav_msgs::msg::Odometry OdomPublisher::output(const AckermannState & state)
{
  nav_msgs::msg::Odometry odom;
  
  // Header
  odom.header.stamp = state.time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "";

  // Pose
  odom.pose.pose.position.x = state.x;
  odom.pose.pose.position.y = state.y;
  odom.pose.pose.position.z = state.z;
  odom.pose.pose.orientation = tf2::toMsg(state.orientation);

  // Twist
  double linear_speed = wheel_radius_ * (state.left_wheel_speed + state.right_wheel_speed) / 2.0;
  double vx, vy, vz;
  linear_velocity(state.orientation, linear_speed, vx, vy, vz);
  
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.linear.z = vz;
  
  double angular_speed = linear_speed / turn_radius(state.steering_angle);
  odom.twist.twist.angular.z = angular_speed;

  return odom;
}

double OdomPublisher::turn_radius(double steering_angle)
{
  // If the steering angle is 0, then cot(0) is undefined
  if (steering_angle == 0.0) {
    return std::numeric_limits<double>::infinity();
  } else {
    double cot_angle = 1.0 / std::tan(steering_angle);
    double radius = std::sqrt(
      center_of_mass_offset_ * center_of_mass_offset_ + 
      wheelbase_length_ * wheelbase_length_ * cot_angle * cot_angle
    );
    return std::copysign(radius, steering_angle);
  }
}

void OdomPublisher::linear_velocity(
  const tf2::Quaternion & orientation, double speed,
  double & vx, double & vy, double & vz)
{
  // Rotate from +x direction
  tf2::Vector3 velocity_vector(speed, 0.0, 0.0);
  tf2::Matrix3x3 rotation_matrix(orientation);
  tf2::Vector3 rotated_velocity = rotation_matrix * velocity_vector;
  
  vx = rotated_velocity.x();
  vy = rotated_velocity.y();
  vz = rotated_velocity.z();
}

}  // namespace ackermann_odometry
