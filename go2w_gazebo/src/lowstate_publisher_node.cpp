#include "go2w_gazebo/lowstate_publisher_node.hpp"

namespace go2w_gazebo
{
LowStatePublisherNode::LowStatePublisherNode() : Node("lowstate_publisher")
{
  // Configure startup-only parameters that control validation behavior.
  rcl_interfaces::msg::ParameterDescriptor fixed;
  fixed.read_only = true;
  check_only_ = declare_parameter("check_only", false, fixed);

  const double timeout = declare_parameter("startup_timeout", 60.0, fixed);

  if (!std::isfinite(timeout) || timeout <= 0) {
    throw std::invalid_argument("Invalid startup_timeout");
  }

  // Use reliable, volatile QoS for native telemetry and feedback topics.
  const auto qos = rclcpp::QoS(10).reliable().durability_volatile();

  if (check_only_) {
    // In check-only mode, wait for a valid native LowState message and exit.
    readiness_ = create_subscription<unitree_go::msg::LowState>(
      "/lowstate", qos,
      std::bind(&LowStatePublisherNode::on_readiness, this, std::placeholders::_1));

  } else {
    // Configure normal publication mode and its simulation feedback sources.
    require_imu_ = declare_parameter("body_imu_enabled", true, fixed);

    const double full_rate = declare_parameter("publish_rate", 200.0, fixed);
    const double low_rate = declare_parameter("lf_publish_rate", 50.0, fixed);

    if (!std::isfinite(full_rate) || !std::isfinite(low_rate) || full_rate <= 0 ||
        full_rate > 1000 || low_rate <= 0 || low_rate > full_rate) {
      throw std::invalid_argument("Require 0 < lf_publish_rate <= publish_rate <= 1000");
    }

    // Declare all adjustable telemetry parameters and validate their defaults.
    for (const auto& spec : utils::telemetry_specs()) {
      declare_parameter(spec.initial.get_name(), spec.initial.get_parameter_value());
      names_.push_back(spec.initial.get_name());

      const auto reason = utils::validate_telemetry(get_parameter(spec.initial.get_name()));

      if (!reason.empty()) {
        throw std::invalid_argument(spec.initial.get_name() + ": " + reason);
      }
    }

    // Reject invalid runtime telemetry changes before they reach publications.
    parameters_callback_ = add_on_set_parameters_callback(
      std::bind(&LowStatePublisherNode::on_parameters, this, std::placeholders::_1));

    full_ = create_publisher<unitree_go::msg::LowState>("/lowstate", qos);

    low_ = create_publisher<unitree_go::msg::LowState>("/lf/lowstate", qos);

    // Subscribe to the simulated joint feedback used for native motor fields.
    joints_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", qos,
      std::bind(&LowStatePublisherNode::on_joint_state, this, std::placeholders::_1));

    if (require_imu_) {
      // Use the simulated body IMU when it is enabled.
      imu_ = create_subscription<unitree_go::msg::IMUState>(
        "/imu", qos, std::bind(&LowStatePublisherNode::on_imu, this, std::placeholders::_1));
    } else {
      imu_state_.quaternion = {1, 0, 0, 0};
      imu_state_.temperature = 25;
    }

    // Publish complete and reduced-rate snapshots using simulation time.
    full_timer_ =
      rclcpp::create_timer(this, get_clock(), std::chrono::duration<double>(1.0 / full_rate),
                           std::bind(&LowStatePublisherNode::publish, this));

    low_timer_ =
      rclcpp::create_timer(this, get_clock(), std::chrono::duration<double>(1.0 / low_rate),
                           std::bind(&LowStatePublisherNode::publish_low_rate, this));
  }

  // Monitor startup and terminate with an error if required feedback is missing.
  const auto timeout_duration = std::chrono::duration<double>(timeout);

  const auto deadline =
    std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(timeout_duration);

  using StartupCallback = std::function<void()>;

  const StartupCallback startup_callback =
    std::bind(&LowStatePublisherNode::check_startup, this, deadline);

  watchdog_ = create_wall_timer(std::chrono::milliseconds(50), startup_callback);
}

void LowStatePublisherNode::on_readiness(unitree_go::msg::LowState::ConstSharedPtr msg)
{
  // Validate the native IMU payload before accepting the readiness message.
  if (!utils::valid_native_imu(msg->imu_state)) {
    return;
  }

  // Validate all native motor values supplied by the readiness source.
  for (const auto& motor : msg->motor_state) {
    for (float value : {motor.q, motor.dq, motor.ddq, motor.tau_est}) {
      if (!std::isfinite(value)) {
        return;
      }
    }
  }

  if (now().nanoseconds() <= 0) {
    return;
  }

  // Signal successful readiness and stop the check-only process.
  RCLCPP_INFO(get_logger(), "LowState feedback is ready");
  started_ = true;
  rclcpp::shutdown();
}

rcl_interfaces::msg::SetParametersResult LowStatePublisherNode::on_parameters(
  const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // Validate only parameters owned by this node and reject the transaction as
  // a whole when any requested telemetry value is invalid.
  for (const auto& parameter : parameters) {
    if (std::find(names_.begin(), names_.end(), parameter.get_name()) == names_.end()) {
      continue;
    }

    const auto reason = utils::validate_telemetry(parameter);

    if (!reason.empty()) {
      result.successful = false;
      result.reason = parameter.get_name() + ": " + reason;
      break;
    }
  }

  // No state is changed here: rejected atomic transactions cannot leak into publications.
  return result;
}

void LowStatePublisherNode::on_joint_state(sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  // Reset stale derivative history when simulation time moves backward.
  observe_time();

  // Convert valid named joint feedback into the native motor ordering.
  if (!feedback_.update(*msg)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejected malformed joint feedback");
  }
}

void LowStatePublisherNode::on_imu(unitree_go::msg::IMUState::ConstSharedPtr msg)
{
  // Keep IMU state synchronized with simulation time and reject invalid samples.
  observe_time();

  if (!utils::valid_native_imu(*msg)) {
    return;
  }

  // Store the latest valid IMU sample and its simulation timestamp.
  imu_state_ = *msg;
  imu_stamp_ = now().nanoseconds();
  have_imu_ = true;
}

void LowStatePublisherNode::publish_low_rate()
{
  // Publish the latest full-rate snapshot when all required feedback is fresh.
  observe_time();

  if (have_snapshot_ && fresh()) {
    low_->publish(snapshot_);
  }
}

void LowStatePublisherNode::check_startup(std::chrono::steady_clock::time_point deadline)
{
  // Stop checking once normal publication or check-only readiness has succeeded.
  if (started_) {
    watchdog_->cancel();
    return;
  }

  // Report missing simulation feedback after the bounded startup interval.
  if (std::chrono::steady_clock::now() >= deadline) {
    RCLCPP_ERROR(get_logger(),
                 "LowState startup timed out: require /clock, valid 16-joint "
                 "position/velocity/effort feedback and the enabled body /imu");
    failed_ = true;
    rclcpp::shutdown();
  }
}

bool LowStatePublisherNode::failed() const
{
  return failed_;
}

void LowStatePublisherNode::observe_time()
{
  const auto time = now().nanoseconds();

  // Clear feedback derived from the previous timeline after a simulation reset.
  if (time < last_time_) {
    feedback_.reset();
    have_imu_ = false;
    have_snapshot_ = false;
  }

  last_time_ = time;
}

bool LowStatePublisherNode::fresh() const
{
  const auto time = now().nanoseconds();

  // Require valid joint feedback and, when enabled, a recent body IMU sample.
  return time > 0 && feedback_.fresh(time) &&
         (!require_imu_ || (have_imu_ && time >= imu_stamp_ && time - imu_stamp_ <= 1000000000));
}

void LowStatePublisherNode::publish()
{
  // Do not publish telemetry while required simulation feedback is stale.
  observe_time();

  if (!fresh()) {
    if (started_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "LowState publication suspended: missing or stale joint/IMU feedback");
    }
    return;
  }

  // Assemble configurable telemetry and overlay live motor simulation values.
  auto msg = utils::telemetry_message(get_parameters(names_));

  for (size_t i = 0; i < 16; ++i) {
    const auto temperature = msg.motor_state[i].temperature;
    const auto mode = msg.motor_state[i].mode;
    const auto lost = msg.motor_state[i].lost;

    msg.motor_state[i] = feedback_.motors[i];
    msg.motor_state[i].temperature = temperature;
    msg.motor_state[i].mode = mode;
    msg.motor_state[i].lost = lost;
  }

  // Add the live IMU state and a simulation-time millisecond tick.
  msg.imu_state = imu_state_;
  msg.tick = static_cast<uint32_t>(now().nanoseconds() / 1000000);

  snapshot_ = msg;

  have_snapshot_ = true;

  // Publish the full-rate message and mark startup as complete.
  full_->publish(msg);

  if (!started_) {
    RCLCPP_INFO(get_logger(), "Publishing native LowState telemetry");
  }

  started_ = true;
}
} // namespace go2w_gazebo

int main(int argc, char** argv)
{
  // Initialize ROS, run the publisher node, and propagate startup failures.
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<go2w_gazebo::LowStatePublisherNode>();

    rclcpp::spin(node);

    int result = 0;

    if (node->failed()) {
      result = 1;
    }

    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }

    return result;
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("lowstate_publisher"), "%s", error.what());

    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }

    return 1;
  }
}
