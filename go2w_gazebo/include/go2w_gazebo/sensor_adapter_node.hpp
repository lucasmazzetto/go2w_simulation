#pragma once

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unitree_go/msg/imu_state.hpp>

#include "go2w_gazebo/utils.hpp"

namespace go2w_gazebo
{
class SensorAdapterNode : public rclcpp::Node
{
public:
  /**
   * @brief Creates the sensor adapter node and its ROS interfaces.
   */
  SensorAdapterNode();

  /**
   * @brief Reports whether startup validation has failed.
   * @return True when required sensor feedback did not become ready in time.
   */
  bool failed() const;

private:
  /**
   * @brief Validates a runtime sensor-temperature parameter.
   * @param parameter Parameter value to validate.
   * @return True when the parameter is a finite integer temperature.
   */
  static bool valid_temperature(const rclcpp::Parameter& parameter);

  /**
   * @brief Validates runtime sensor parameter updates.
   * @param parameters Parameters being changed.
   * @return Result indicating whether the update is accepted.
   */
  rcl_interfaces::msg::SetParametersResult on_parameters(
    const std::vector<rclcpp::Parameter>& parameters);

  /**
   * @brief Converts and publishes one IMU message.
   * @param index Sensor index used to select the output publisher.
   * @param frame Output frame assigned to the message.
   * @param gyro Gyroscope covariance value.
   * @param acceleration Accelerometer covariance value.
   * @param msg Incoming IMU message.
   */
  void on_imu(size_t index, const std::string& frame, double gyro, double acceleration,
              sensor_msgs::msg::Imu::UniquePtr msg);

  /**
   * @brief Converts and publishes one point cloud and its public alias.
   * @param frame Output frame assigned to the cloud.
   * @param minimum Minimum accepted point range.
   * @param maximum Maximum accepted point range.
   * @param msg Incoming point cloud.
   */
  void on_cloud(const std::string& frame, double minimum, double maximum,
                sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  /**
   * @brief Validates camera image metadata and updates readiness state.
   * @param frame Expected image frame.
   * @param width Expected image width in pixels.
   * @param height Expected image height in pixels.
   * @param msg Incoming camera image.
   */
  void on_camera_image(const std::string& frame, uint32_t width, uint32_t height,
                       sensor_msgs::msg::Image::ConstSharedPtr msg);

  /**
   * @brief Validates camera calibration and updates readiness state.
   * @param frame Expected optical frame.
   * @param width Expected image width in pixels.
   * @param height Expected image height in pixels.
   * @param intrinsics Expected intrinsic parameters.
   * @param msg Incoming camera calibration message.
   */
  void on_camera_info(const std::string& frame, uint32_t width, uint32_t height,
                      const std::vector<double>& intrinsics,
                      sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);

  /**
   * @brief Checks whether all required sensor topics have produced valid data.
   * @return True when every enabled sensor is ready.
   */
  bool all_sensors_seen() const;

  /**
   * @brief Checks whether required sensor data arrives before the deadline.
   * @param deadline Maximum steady-clock time allowed for startup.
   * @param names Names used to report missing sensors.
   */
  void check_startup(std::chrono::steady_clock::time_point deadline,
                     const std::array<std::string, 5>& names);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr temperature_callback_;

  bool check_only_{false};
  bool failed_{false};
  std::array<bool, 5> seen_{};
  bool camera_ready_{true};

  builtin_interfaces::msg::Time camera_image_stamp_;
  builtin_interfaces::msg::Time camera_info_stamp_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;

  std::array<rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr, 2> imu_publishers_;

  rclcpp::Publisher<unitree_go::msg::IMUState>::SharedPtr native_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr alias_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr> imu_subscriptions_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace go2w_gazebo
