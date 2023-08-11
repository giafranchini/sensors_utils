#ifndef IMU_UTILS_IMU_PROCESSOR_HPP__
#define IMU_UTILS_IMU_PROCESSOR_HPP__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <chrono>
#include <vector>

namespace imu_utils
{
  using namespace std::chrono_literals;
  class ImuProcessor : public rclcpp::Node
  {
  public:
    explicit ImuProcessor(const rclcpp::NodeOptions& options);
  
  private:
    void processor_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_;

    std::vector<double> orientation_covariance_;
    std::vector<double> angular_velocity_covariance_;
    std::vector<double> linear_acceleration_covariance_;
    bool compute_covariance_;
  };

}  // namespace imu_utils

#endif  // IMU_UTILS_IMU_PROCESSOR_HPP__
