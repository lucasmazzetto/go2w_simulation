#include "go2w_gazebo/cmd_vel_adapter_node.hpp"

namespace go2w_gazebo
{
CmdVelAdapterNode::CmdVelAdapterNode() : Node("cmd_vel_adapter")
{
  // Publish stamped commands for the drive controller.
  publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>("drive_controller/cmd_vel", 10);

  // Accept the public unstamped command interface.
  subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&CmdVelAdapterNode::forward, this, std::placeholders::_1));
}

void CmdVelAdapterNode::forward(const geometry_msgs::msg::Twist::SharedPtr command)
{
  // Reject invalid forward and yaw values before they reach the controller.
  if (!std::isfinite(command->linear.x) || !std::isfinite(command->angular.z)) {
    RCLCPP_WARN(get_logger(), "Ignoring non-finite forward or yaw velocity");
    return;
  }

  // Do not publish until Gazebo has supplied a simulation timestamp.
  const auto stamp = now();

  if (stamp.nanoseconds() == 0) {
    RCLCPP_WARN(get_logger(), "Waiting for the simulation clock before forwarding commands");
    return;
  }

  // Preserve the supported differential-drive components and stamp once.
  geometry_msgs::msg::TwistStamped stamped;
  stamped.header.stamp = stamp;
  stamped.header.frame_id = "base";
  stamped.twist.linear.x = command->linear.x;
  stamped.twist.angular.z = command->angular.z;

  publisher_->publish(stamped);
}
} // namespace go2w_gazebo

int main(int argc, char** argv)
{
  // Start the command adapter node.
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2w_gazebo::CmdVelAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
