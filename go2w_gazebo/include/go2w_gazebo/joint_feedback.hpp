#pragma once

#include "go2w_gazebo/utils.hpp"

namespace go2w_gazebo
{
/**
 * @brief Converts named simulated joint feedback into native motor feedback.
 */
class JointFeedback
{
public:
  /**
   * @brief Updates all native motor fields from one joint-state message.
   * @param msg Joint-state message containing simulated feedback.
   * @return True when feedback for all required joints is valid.
   */
  bool update(const sensor_msgs::msg::JointState& msg);

  /**
   * @brief Checks whether the most recent joint feedback is still usable.
   * @param now Current simulation timestamp in nanoseconds.
   * @return True when feedback is valid and no older than the freshness limit.
   */
  bool fresh(int64_t now) const;

  /**
   * @brief Clears feedback and derivative history after a simulation-time reset.
   */
  void reset();

  /** Native motor values. */
  std::array<unitree_go::msg::MotorState, 20> motors;

  /** Whether complete feedback has been accepted. */
  bool valid{false};

private:
  int64_t stamp_ns_{0};
  std::array<double, 16> velocities_{};
};
} // namespace go2w_gazebo
