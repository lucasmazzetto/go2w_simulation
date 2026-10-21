#pragma once

#include <cmath>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace go2w_gazebo
{
/**
 * @brief Converts unstamped velocity commands to drive-controller commands.
 *
 * @details Each valid command is forwarded once with the current simulation
 * timestamp. Only forward velocity and yaw rate are used by the differential
 * drive controller.
 */
class CmdVelAdapterNode : public rclcpp::Node
{
public:
  /**
   * @brief Creates the /cmd_vel adapter node and its ROS interfaces.
   */
  CmdVelAdapterNode();

private:
  /**
   * @brief Validates and forwards one unstamped velocity command.
   *
   * @param command Command containing the requested forward velocity and yaw rate.
   */
  void forward(const geometry_msgs::msg::Twist::SharedPtr command);
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};
} // namespace go2w_gazebo
