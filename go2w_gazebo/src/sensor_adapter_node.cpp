#include "go2w_gazebo/sensor_adapter_node.hpp"

namespace go2w_gazebo
{
// Adapt Gazebo sensor topics to the public Go2-W ROS interfaces.
SensorAdapterNode::SensorAdapterNode() : Node("sensor_adapter")
{
  // Startup mode validates all enabled sensors before the simulation is used.
  check_only_ = declare_parameter("check_only", false);

  // Public sensor publishers use reliable, volatile QoS.
  const auto public_qos = rclcpp::QoS(10).reliable().durability_volatile();

  // Keep sensor names, frames, and public topics in one mapping.
  const std::array<std::string, 5> names = {"body_imu", "lidar_imu", "lidar", "camera image",
                                            "camera info"};

  const std::array<std::string, 3> frames = {"imu", "utlidar_imu", "radar"};

  const std::array<std::string, 3> topics = {"/imu/data", "/utlidar/imu", "/utlidar/cloud"};

  // Validate the initial temperature before registering its runtime callback.
  declare_parameter("temperature", 25);

  if (!valid_temperature(get_parameter("temperature"))) {
    throw std::invalid_argument("temperature must fit native int8 degrees Celsius");
  }

  temperature_callback_ = add_on_set_parameters_callback(
    std::bind(&SensorAdapterNode::on_parameters, this, std::placeholders::_1));

  const double minimum = declare_parameter("range_min", 0.05);
  const double maximum = declare_parameter("range_max", 20.0);

  // Configure the two IMU inputs and the LiDAR input from the shared mappings.
  for (size_t i = 0; i < topics.size(); ++i) {
    // Disabled sensors are already ready for the startup check.
    const bool enabled = declare_parameter(names[i] + "_enabled", true);
    const auto frame = declare_parameter(names[i] + "_frame", frames[i]);

    seen_[i] = !enabled;

    if (i < 2) {
      // Configure the two IMU adapters.
      const double gyro = declare_parameter(names[i] + "_gyroscope_noise_stddev", 0.0);
      const double acceleration = declare_parameter(names[i] + "_accelerometer_noise_stddev", 0.0);

      if (!enabled) {
        continue;
      }

      if (!check_only_) {
        // The body IMU also exposes the native Unitree message.
        if (i == 0) {
          native_ = create_publisher<unitree_go::msg::IMUState>("/imu", public_qos);
        }
        imu_publishers_[i] = create_publisher<sensor_msgs::msg::Imu>(topics[i], public_qos);
      }

      auto input_topic = topics[i];
      auto input_qos = public_qos;

      if (!check_only_) {
        input_topic = "/go2w_gazebo/sensors/" + names[i];
        input_qos = rclcpp::SensorDataQoS();
      }

      // Use the internal Gazebo topic during normal operation and the public
      // topic directly when validating an existing world.
      using ImuCallback = std::function<void(sensor_msgs::msg::Imu::UniquePtr)>;
      const ImuCallback callback = std::bind(&SensorAdapterNode::on_imu, this, i, frame, gyro,
                                             acceleration, std::placeholders::_1);
      imu_subscriptions_.push_back(
        create_subscription<sensor_msgs::msg::Imu>(input_topic, input_qos, callback));
    } else if (enabled) {
      // Configure the formatted LiDAR cloud and its compatibility alias.
      if (!check_only_) {
        cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(topics[i], public_qos);
        alias_ = create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud", public_qos);
      }

      auto input_topic = topics[i];
      auto input_qos = public_qos;

      if (!check_only_) {
        input_topic = "/go2w_gazebo/sensors/lidar";
        input_qos = rclcpp::SensorDataQoS();
      }

      using CloudCallback = std::function<void(sensor_msgs::msg::PointCloud2::ConstSharedPtr)>;
      const CloudCallback callback = std::bind(&SensorAdapterNode::on_cloud, this, frame, minimum,
                                               maximum, std::placeholders::_1);
      cloud_subscription_ =
        create_subscription<sensor_msgs::msg::PointCloud2>(input_topic, input_qos, callback);
    }
  }

  // Configure camera validation only when camera startup checking is requested.
  const bool camera_enabled = declare_parameter("camera_enabled", false);

  seen_[3] = seen_[4] = !camera_enabled;
  camera_ready_ = !camera_enabled;

  if (check_only_ && camera_enabled) {
    const auto frame = declare_parameter("camera_frame", "front_camera_optical");

    const auto width = static_cast<uint32_t>(declare_parameter("camera_width", 1280));

    const auto height = static_cast<uint32_t>(declare_parameter("camera_height", 720));

    const auto intrinsics =
      declare_parameter<std::vector<double>>("camera_intrinsics", std::vector<double>{});

    const auto image_topic = declare_parameter("camera_image_topic", "/camera/image_raw");

    const auto info_topic = declare_parameter("camera_info_topic", "/camera/camera_info");

    using ImageCallback = std::function<void(sensor_msgs::msg::Image::ConstSharedPtr)>;
    const ImageCallback image_callback = std::bind(&SensorAdapterNode::on_camera_image, this, frame,
                                                   width, height, std::placeholders::_1);
    camera_image_subscription_ =
      create_subscription<sensor_msgs::msg::Image>(image_topic, public_qos, image_callback);

    using CameraInfoCallback = std::function<void(sensor_msgs::msg::CameraInfo::ConstSharedPtr)>;
    const CameraInfoCallback info_callback =
      std::bind(&SensorAdapterNode::on_camera_info, this, frame, width, height, intrinsics,
                std::placeholders::_1);
    camera_info_subscription_ =
      create_subscription<sensor_msgs::msg::CameraInfo>(info_topic, public_qos, info_callback);
  }

  // Bound startup validation with a wall-clock watchdog.
  if (check_only_) {
    // Bound readiness checks by wall time instead of relying on a fixed sleep.
    const double timeout = declare_parameter("startup_timeout", 60.0);
    const auto timeout_duration = std::chrono::duration<double>(timeout);
    const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(timeout_duration);

    using StartupCallback = std::function<void()>;
    const StartupCallback startup_callback =
      std::bind(&SensorAdapterNode::check_startup, this, deadline, names);

    timer_ = create_wall_timer(std::chrono::milliseconds(50), startup_callback);
  }
}

bool SensorAdapterNode::valid_temperature(const rclcpp::Parameter& parameter)
{
  // Native telemetry stores temperature as a signed 8-bit integer.
  return parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER &&
         parameter.as_int() >= -128 && parameter.as_int() <= 127;
}

rcl_interfaces::msg::SetParametersResult SensorAdapterNode::on_parameters(
  const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // Reject invalid temperature updates without affecting valid parameters.
  for (const auto& parameter : parameters) {
    if (parameter.get_name() == "temperature" && !valid_temperature(parameter)) {
      result.successful = false;
      result.reason = "temperature must fit native int8 degrees Celsius";
    }
  }
  return result;
}

void SensorAdapterNode::on_imu(size_t index, const std::string& frame, double gyro,
                               double acceleration, sensor_msgs::msg::Imu::UniquePtr msg)
{
  // Ignore malformed messages and measurements from an unexpected frame.
  if (!utils::valid_imu(*msg) || msg->header.frame_id != frame) {
    return;
  }

  seen_[index] = true;
  if (check_only_) {
    return;
  }

  // Apply configured covariance values before publishing the public IMU topic.
  utils::set_covariances(*msg, gyro, acceleration);
  imu_publishers_[index]->publish(*msg);

  if (index == 0) {
    // The body IMU also feeds the native Unitree telemetry topic.
    native_->publish(utils::native_imu(*msg, get_parameter("temperature").as_int()));
  }
}

void SensorAdapterNode::on_cloud(const std::string& frame, double minimum, double maximum,
                                 sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  try {
    // Formatting checks the field layout and data, including empty clouds.
    auto output = utils::format_cloud(*msg, frame, minimum, maximum);

    if (check_only_) {
      // Startup validation only needs to confirm the expected input frame.
      if (msg->header.frame_id == frame) {
        seen_[2] = true;
      }
      return;
    }

    // Publish the formatted cloud on both the native-compatible and alias topics.
    seen_[2] = true;
    cloud_->publish(output);
    alias_->publish(output);

  } catch (const std::exception& error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
  }
}

void SensorAdapterNode::on_camera_image(const std::string& frame, uint32_t width, uint32_t height,
                                        sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  // Accept only images matching the configured camera layout and frame.
  if (!utils::valid_camera_image(*msg, frame, width, height)) {
    return;
  }

  seen_[3] = true;
  // A camera is ready only after image and CameraInfo timestamps match.
  camera_image_stamp_ = msg->header.stamp;
  camera_ready_ = camera_ready_ || (seen_[4] && camera_image_stamp_ == camera_info_stamp_);
}

void SensorAdapterNode::on_camera_info(const std::string& frame, uint32_t width, uint32_t height,
                                       const std::vector<double>& intrinsics,
                                       sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  // Accept only calibration matching the configured camera layout and intrinsics.
  if (!utils::valid_camera_info(*msg, frame, width, height, intrinsics)) {
    return;
  }

  seen_[4] = true;
  // A camera is ready only after CameraInfo and image timestamps match.
  camera_info_stamp_ = msg->header.stamp;
  camera_ready_ = camera_ready_ || (seen_[3] && camera_image_stamp_ == camera_info_stamp_);
}

bool SensorAdapterNode::all_sensors_seen() const
{
  // Disabled sensors are initialized as seen, so only enabled inputs can block startup.
  for (const bool seen : seen_) {
    if (!seen) {
      return false;
    }
  }

  return true;
}

void SensorAdapterNode::check_startup(std::chrono::steady_clock::time_point deadline,
                                      const std::array<std::string, 5>& names)
{
  // Start only after each enabled sensor and simulation clock are valid.
  if (all_sensors_seen() && camera_ready_ && now().nanoseconds() > 0) {
    RCLCPP_INFO(get_logger(), "Enabled sensors are ready; driving can start");
    rclcpp::shutdown();
  } else if (std::chrono::steady_clock::now() >= deadline) {
    // Report missing sensor streams and camera synchronization separately.
    for (size_t i = 0; i < names.size(); ++i) {
      if (!seen_[i]) {
        RCLCPP_ERROR(get_logger(), "No valid %s measurement", names[i].c_str());
      }
    }

    if (!camera_ready_ && seen_[3] && seen_[4]) {
      RCLCPP_ERROR(get_logger(), "No matching camera image/CameraInfo timestamps");
    }

    if (now().nanoseconds() == 0) {
      RCLCPP_ERROR(get_logger(),
                   "No simulation time on /clock. Bridge the existing world's "
                   "clock or set bridge_clock:=true.");
    }

    RCLCPP_ERROR(
      get_logger(),
      "Sensor startup timed out. Check the world's Imu and Sensors systems, "
      "OGRE2/EGL rendering, clock and bridge. Use enable_sensors:=false for physics only.");

    failed_ = true;
    rclcpp::shutdown();
  }
}

// Return the result of the bounded startup check to the launch process.
bool SensorAdapterNode::failed() const
{
  return failed_;
}
} // namespace go2w_gazebo

int main(int argc, char** argv)
{
  // Start the sensor adapter node and return a failure status when validation fails.
  rclcpp::init(argc, argv);

  auto node = std::make_shared<go2w_gazebo::SensorAdapterNode>();

  rclcpp::spin(node);

  int result = 0;

  if (node->failed()) {
    result = 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return result;
}
