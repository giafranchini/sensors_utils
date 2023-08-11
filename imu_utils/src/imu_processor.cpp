#include "imu_utils/imu_processor.hpp"

#include <memory>

#include <vector>
#include <iostream>
#include <chrono>

namespace imu_utils
{

ImuProcessor::ImuProcessor(const rclcpp::NodeOptions& options) : rclcpp::Node("imu_processor", options)
{
  subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu_in", rclcpp::SensorDataQoS(),
      std::bind(&ImuProcessor::processor_callback, this, std::placeholders::_1));

  publisher_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_out", 10);
  RCLCPP_INFO(this->get_logger(), "Subscribed to imu topic");
 
  declare_parameter("orientation_covariance", std::vector<double>(9, 0.0));
  declare_parameter("angular_velocity_covariance", std::vector<double>(9, 0.0));
  declare_parameter("linear_acceleration_covariance", std::vector<double>(9, 0.0));
  declare_parameter("compute_covariance", false);

  orientation_covariance_ = get_parameter("orientation_covariance").as_double_array();
  angular_velocity_covariance_ = get_parameter("angular_velocity_covariance").as_double_array();
  linear_acceleration_covariance_ = get_parameter("linear_acceleration_covariance").as_double_array();
  compute_covariance_ = get_parameter("compute_covariance").as_bool();

  RCLCPP_INFO(this->get_logger(), "Imu processor initialization finished");
}

void ImuProcessor::processor_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  auto processed_msg = std::make_unique<sensor_msgs::msg::Imu>();

  if (compute_covariance_)
  {
    RCLCPP_WARN(this->get_logger(), "TODO: IMPLEMENT A METHOD TO COMPUTE THE DESIDERED COVARIANCE");
    return;
  }
  
  if ((msg->orientation.x == 0) && (msg->orientation.y == 0) && (msg->orientation.z == 0) && (msg->orientation.w == 0.0))
  {
    processed_msg->orientation_covariance[0] = -1;
  } else
  {
    for (size_t i = 0; i < 9; i++) {processed_msg->orientation_covariance[i] = orientation_covariance_[i];};
  }

  if ((msg->angular_velocity.x == 0) && (msg->angular_velocity.y == 0) && (msg->angular_velocity.z == 0))
  {
    processed_msg->angular_velocity_covariance[0] = -1;
  } else
  {
    for (size_t i = 0; i < 9; i++) {processed_msg->angular_velocity_covariance[i] = angular_velocity_covariance_[i];};
  }

  if ((msg->linear_acceleration.x == 0 ) && (msg->linear_acceleration.y == 0) && (msg->linear_acceleration.z == 0))
  {
    processed_msg->linear_acceleration_covariance[0] = -1;
  } else
  {
    for (size_t i = 0; i < 9; i++) {processed_msg->linear_acceleration_covariance[i] = linear_acceleration_covariance_[i];};
  }
  
  processed_msg->header = msg->header;
  processed_msg->orientation = msg->orientation;
  processed_msg->angular_velocity = msg->angular_velocity;
  processed_msg->linear_acceleration = msg->linear_acceleration;
  publisher_imu_->publish(std::move(processed_msg));
  
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  RCLCPP_DEBUG(
    get_logger(), "Time difference = %ld [ms]",
    std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());
}

}  // namespace imu_utils

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(imu_utils::ImuProcessor)

