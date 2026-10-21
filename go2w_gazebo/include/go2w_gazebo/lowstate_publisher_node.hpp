#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <algorithm>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unitree_go/msg/imu_state.hpp>
#include <unitree_go/msg/low_state.hpp>

#include "go2w_gazebo/joint_feedback.hpp"

namespace go2w_gazebo
{
class LowStatePublisherNode : public rclcpp::Node
{
public:
  /**
   * @brief Creates the LowState publisher node and its ROS interfaces.
   */
  LowStatePublisherNode();

  /**
   * @brief Reports whether startup validation has failed.
   * @return True when required feedback did not become ready in time.
   */
  bool failed() const;

private:
  /**
   * @brief Detects simulation-time resets and clears stale state.
   */
  void observe_time();

  /**
   * @brief Checks whether all required feedback is current.
   * @return True when joint and required IMU feedback is fresh.
   */
  bool fresh() const;

  /**
   * @brief Publishes the latest full-rate LowState snapshot.
   */
  void publish();

  /**
   * @brief Publishes the latest snapshot on the lower-rate topic.
   */
  void publish_low_rate();

  /**
   * @brief Records native LowState feedback during startup validation.
   * @param msg Native LowState message received from the simulation.
   */
  void on_readiness(unitree_go::msg::LowState::ConstSharedPtr msg);

  /**
   * @brief Updates motor feedback from the simulation joint states.
   * @param msg Simulation joint-state message.
   */
  void on_joint_state(sensor_msgs::msg::JointState::ConstSharedPtr msg);

  /**
   * @brief Updates the native IMU state used in LowState messages.
   * @param msg Native IMU message received from the sensor adapter.
   */
  void on_imu(unitree_go::msg::IMUState::ConstSharedPtr msg);

  /**
   * @brief Validates runtime telemetry parameter updates.
   * @param parameters Parameters being changed.
   * @return Result indicating whether the update is accepted.
   */
  rcl_interfaces::msg::SetParametersResult on_parameters(
    const std::vector<rclcpp::Parameter>& parameters);

  /**
   * @brief Checks whether required feedback arrives before the deadline.
   * @param deadline Maximum steady-clock time allowed for startup.
   */
  void check_startup(std::chrono::steady_clock::time_point deadline);

  bool check_only_{false};
  bool require_imu_{true};
  bool started_{false};
  bool failed_{false};
  bool have_imu_{false};
  bool have_snapshot_{false};
  int64_t last_time_{0};
  int64_t imu_stamp_{0};
  JointFeedback feedback_;
  unitree_go::msg::IMUState imu_state_;
  unitree_go::msg::LowState snapshot_;
  std::vector<std::string> names_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  rclcpp::Publisher<unitree_go::msg::LowState>::SharedPtr full_;
  rclcpp::Publisher<unitree_go::msg::LowState>::SharedPtr low_;

  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr readiness_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Subscription<unitree_go::msg::IMUState>::SharedPtr imu_;

  rclcpp::TimerBase::SharedPtr full_timer_;
  rclcpp::TimerBase::SharedPtr low_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};
} // namespace go2w_gazebo
