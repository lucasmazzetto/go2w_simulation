#include "go2w_gazebo/joint_feedback.hpp"

namespace go2w_gazebo
{
bool JointFeedback::update(const sensor_msgs::msg::JointState& msg)
{
  // Validate the joint-state array sizes and simulation timestamp first.
  const auto count = msg.name.size();

  const bool arrays_match =
    msg.position.size() == count && msg.velocity.size() == count && msg.effort.size() == count;

  const bool timestamp_is_valid =
    msg.header.stamp.sec >= 0 && msg.header.stamp.nanosec < 1000000000u;

  if (!arrays_match || !timestamp_is_valid) {
    return false;
  }

  // Index the incoming joints by name so the native motor order is independent
  // of the order used by the joint-state publisher.
  std::map<std::string, size_t> indices;

  for (size_t i = 0; i < count; ++i) {
    // Reject duplicate joint names and values that cannot be represented natively.
    const bool name_is_unique = indices.emplace(msg.name[i], i).second;
    const bool values_are_valid = utils::native_float(msg.position[i]) &&
                                  utils::native_float(msg.velocity[i]) &&
                                  utils::native_float(msg.effort[i]);

    if (!name_is_unique || !values_are_valid) {
      return false;
    }
  }

  // Require feedback for every motor represented in the native LowState message.
  for (const auto& name : utils::MOTOR_JOINTS) {
    if (indices.count(name) == 0) {
      return false;
    }
  }

  const int64_t stamp = int64_t(msg.header.stamp.sec) * 1000000000 + msg.header.stamp.nanosec;
  const double dt = (stamp - stamp_ns_) * 1e-9;

  // Build a complete next snapshot so invalid input cannot partially change state.
  auto next = motors;

  for (size_t i = 0; i < utils::MOTOR_JOINTS.size(); ++i) {
    // Calculate acceleration from consecutive velocity samples when history is valid.
    const auto j = indices.at(utils::MOTOR_JOINTS[i]);
    double acceleration = 0;

    if (valid && dt > 0) {
      acceleration = (msg.velocity[j] - velocities_[i]) / dt;
    }

    if (!utils::native_float(acceleration)) {
      return false;
    }

    // Copy simulation position, velocity, acceleration, and effort to native fields.
    auto& motor = next[i];

    motor.q = motor.q_raw = static_cast<float>(msg.position[j]);
    motor.dq = motor.dq_raw = static_cast<float>(msg.velocity[j]);
    motor.ddq = motor.ddq_raw = static_cast<float>(acceleration);
    motor.tau_est = static_cast<float>(msg.effort[j]);
  }

  // Preserve the current velocities for the next acceleration calculation.
  for (size_t i = 0; i < utils::MOTOR_JOINTS.size(); ++i) {
    velocities_[i] = msg.velocity[indices.at(utils::MOTOR_JOINTS[i])];
  }

  // Commit the new snapshot and timestamp only after every motor was validated.
  motors = next;
  stamp_ns_ = stamp;
  valid = true;

  return true;
}

bool JointFeedback::fresh(int64_t now) const
{
  // Feedback is fresh only when time has not moved backward and the one-second
  // stale-data limit has not been exceeded.
  return valid && now >= stamp_ns_ && now - stamp_ns_ <= 1000000000;
}

void JointFeedback::reset()
{
  // Clear validity, derivative history, and native motor values after a time reset.
  valid = false;
  stamp_ns_ = 0;
  velocities_.fill(0);
  motors.fill(unitree_go::msg::MotorState{});
}
} // namespace go2w_gazebo
