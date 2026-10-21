#include "go2w_gazebo/utils.hpp"

namespace go2w_gazebo::utils
{
// Validate finite IMU values and ensure the orientation is a unit quaternion.
bool valid_imu(const sensor_msgs::msg::Imu& msg)
{
  const auto& q = msg.orientation;
  const auto& a = msg.linear_acceleration;
  const auto& g = msg.angular_velocity;

  for (double value : {q.w, q.x, q.y, q.z, a.x, a.y, a.z, g.x, g.y, g.z}) {
    if (!std::isfinite(value)) {
      return false;
    }
  }

  const double norm = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;

  return std::abs(norm - 1.0) < 0.01;
}

// Convert a standard ROS IMU message to the native Unitree representation.
unitree_go::msg::IMUState native_imu(const sensor_msgs::msg::Imu& msg, int temperature)
{
  unitree_go::msg::IMUState result;
  const auto& q = msg.orientation;

  result.quaternion = {static_cast<float>(q.w), static_cast<float>(q.x), static_cast<float>(q.y),
                       static_cast<float>(q.z)};

  result.rpy = {
    static_cast<float>(std::atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))),
    static_cast<float>(std::asin(std::clamp(2 * (q.w * q.y - q.z * q.x), -1.0, 1.0))),
    static_cast<float>(std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))};

  result.gyroscope = {static_cast<float>(msg.angular_velocity.x),
                      static_cast<float>(msg.angular_velocity.y),
                      static_cast<float>(msg.angular_velocity.z)};

  result.accelerometer = {static_cast<float>(msg.linear_acceleration.x),
                          static_cast<float>(msg.linear_acceleration.y),
                          static_cast<float>(msg.linear_acceleration.z)};

  result.temperature = static_cast<int8_t>(temperature);

  return result;
}

// Initialize IMU covariance matrices with the configured diagonal variances.
void set_covariances(sensor_msgs::msg::Imu& msg, double gyro, double acceleration)
{
  msg.orientation_covariance.fill(0.0);
  msg.angular_velocity_covariance.fill(0.0);
  msg.linear_acceleration_covariance.fill(0.0);

  for (size_t i : {0u, 4u, 8u}) {
    msg.angular_velocity_covariance[i] = gyro * gyro;
    msg.linear_acceleration_covariance[i] = acceleration * acceleration;
  }
}

// Decode an unsigned integer from point-cloud bytes using the requested byte order.
uint32_t read_bits(const uint8_t* data, size_t length, bool big_endian)
{
  uint32_t result = 0;

  for (size_t i = 0; i < length; ++i) {
    size_t shift = i;

    if (big_endian) {
      shift = length - 1 - i;
    }

    result |= uint32_t(data[i]) << (8 * shift);
  }
  return result;
}

// Decode a 32-bit floating-point value from point-cloud bytes.
float read_float(const uint8_t* data, bool big_endian)
{
  const uint32_t bits = read_bits(data, 4, big_endian);
  float result;

  std::memcpy(&result, &bits, sizeof(result));

  return result;
}

// Encode a 32-bit floating-point value in little-endian byte order.
void write_float(uint8_t* data, float value)
{
  uint32_t bits;

  std::memcpy(&bits, &value, sizeof(bits));

  for (size_t i = 0; i < 4; ++i) {
    data[i] = (bits >> (i * 8)) & 0xff;
  }
}

// Locate a required point-cloud field and verify its layout is safe to read.
uint32_t field_offset(const sensor_msgs::msg::PointCloud2& cloud, const std::string& name,
                      uint8_t type, uint32_t size)
{
  // Find the requested field and verify that it fits inside one point.
  for (const auto& field : cloud.fields) {
    const bool field_matches = field.name == name && field.datatype == type && field.count == 1;
    const bool field_fits = uint64_t(field.offset) + size <= cloud.point_step;

    if (field_matches && field_fits) {
      return field.offset;
    }
  }

  throw std::runtime_error("Missing or invalid point-cloud field: " + name);
}

// Convert the input cloud into the normalized public Go2-W point layout.
sensor_msgs::msg::PointCloud2 format_cloud(const sensor_msgs::msg::PointCloud2& source,
                                           const std::string& frame, double range_min,
                                           double range_max)
{
  using Field = sensor_msgs::msg::PointField;

  // Resolve all required input fields before iterating over point data.
  const std::array<uint32_t, 4> offsets = {field_offset(source, "x", Field::FLOAT32, 4),
                                           field_offset(source, "y", Field::FLOAT32, 4),
                                           field_offset(source, "z", Field::FLOAT32, 4),
                                           field_offset(source, "intensity", Field::FLOAT32, 4)};

  const auto ring_offset = field_offset(source, "ring", Field::UINT16, 2);

  const bool point_step_is_zero = source.point_step == 0;
  const bool row_is_truncated = uint64_t(source.width) * source.point_step > source.row_step;
  const bool data_is_truncated = uint64_t(source.height) * source.row_step > source.data.size();

  if (point_step_is_zero || row_is_truncated || data_is_truncated) {
    throw std::runtime_error("Invalid point-cloud dimensions or truncated data");
  }

  // Initialize the normalized public point-cloud metadata.
  sensor_msgs::msg::PointCloud2 result;
  result.header = source.header;
  result.header.frame_id = frame;
  result.height = 1;
  result.point_step = 24;
  result.is_bigendian = false;
  result.is_dense = true;

  const std::array<std::pair<std::string, uint32_t>, 6> field_layout = {
    {{"x", 0}, {"y", 4}, {"z", 8}, {"intensity", 12}, {"ring", 16}, {"time", 20}}};

  for (const auto& entry : field_layout) {
    Field field;
    field.name = entry.first;
    field.offset = entry.second;

    if (field.name == "ring") {
      field.datatype = Field::UINT16;
    } else {
      field.datatype = Field::FLOAT32;
    }

    field.count = 1;
    result.fields.push_back(field);
  }

  result.data.reserve(uint64_t(source.width) * source.height * result.point_step);

  // Filter invalid or out-of-range points and encode accepted points.
  for (uint32_t row = 0; row < source.height; ++row) {
    for (uint32_t column = 0; column < source.width; ++column) {
      const auto* input =
        source.data.data() + size_t(row) * source.row_step + size_t(column) * source.point_step;

      std::array<float, 4> values;

      for (size_t i = 0; i < values.size(); ++i) {
        values[i] = read_float(input + offsets[i], source.is_bigendian);
      }

      const double range = std::hypot(double(values[0]), double(values[1]), double(values[2]));
      const bool range_is_invalid = !std::isfinite(range) || range < range_min || range > range_max;

      if (range_is_invalid) {
        continue;
      }

      if (!std::isfinite(values[3])) {
        values[3] = 0;
      }

      const auto index = result.data.size();

      result.data.resize(index + result.point_step, 0);

      auto* output = result.data.data() + index;

      for (size_t i = 0; i < values.size(); ++i) {
        write_float(output + 4 * i, values[i]);
      }

      const auto ring = read_bits(input + ring_offset, 2, source.is_bigendian);

      output[16] = ring & 0xff;
      output[17] = (ring >> 8) & 0xff;

      ++result.width;
    }
  }

  result.row_step = result.width * result.point_step;

  return result;
}

// Validate image metadata, encoding, dimensions, and buffer layout.
bool valid_camera_image(const sensor_msgs::msg::Image& image, const std::string& frame,
                        uint32_t width, uint32_t height)
{
  const bool frame_matches = image.header.frame_id == frame;

  const bool has_timestamp = image.header.stamp.sec > 0 || image.header.stamp.nanosec > 0;

  const bool dimensions_match = image.width == width && image.height == height;

  const bool encoding_matches = image.encoding == "rgb8";

  const bool step_is_valid = image.step >= static_cast<uint64_t>(width) * 3;

  const bool buffer_is_complete = image.data.size() == static_cast<uint64_t>(image.step) * height;

  return frame_matches && has_timestamp && dimensions_match && encoding_matches && step_is_valid &&
         buffer_is_complete;
}

// Compare camera calibration values using the configured numerical tolerance.
bool close_enough(double first, double second)
{
  // Camera calibration values use a small absolute comparison tolerance.
  return std::isfinite(first) && std::isfinite(second) && std::abs(first - second) < 1e-5;
}

// Check whether a distortion coefficient is exactly zero.
bool is_zero(double value)
{
  return value == 0;
}

// Validate camera calibration metadata against the configured pinhole model.
bool valid_camera_info(const sensor_msgs::msg::CameraInfo& info, const std::string& frame,
                       uint32_t width, uint32_t height, const std::vector<double>& intrinsics)
{
  const bool frame_matches = info.header.frame_id == frame;

  const bool dimensions_match = info.width == width && info.height == height;

  const bool has_timestamp = info.header.stamp.sec != 0 || info.header.stamp.nanosec != 0;

  const bool model_matches = info.distortion_model == "plumb_bob";

  const bool arrays_have_expected_size = info.d.size() == 5 && intrinsics.size() == 9;

  const bool metadata_is_valid = frame_matches && dimensions_match && has_timestamp &&
                                 model_matches && arrays_have_expected_size;

  if (!metadata_is_valid) {
    return false;
  }

  const std::array<double, 9> identity = {1, 0, 0, 0, 1, 0, 0, 0, 1};

  const std::array<double, 12> projection = {
    intrinsics[0], 0, intrinsics[2], 0, 0, intrinsics[4], intrinsics[5], 0, 0, 0, 1, 0};

  return std::equal(info.k.begin(), info.k.end(), intrinsics.begin(), close_enough) &&
         std::equal(info.r.begin(), info.r.end(), identity.begin(), close_enough) &&
         std::equal(info.p.begin(), info.p.end(), projection.begin(), close_enough) &&
         std::all_of(info.d.begin(), info.d.end(), is_zero);
}

// Check whether a value is finite and representable by a native float field.
bool native_float(double value)
{
  return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();
}

// Build the immutable defaults and native bounds once, independently of live parameters.
const std::vector<TelemetrySpec>& telemetry_specs()
{
  static const std::vector<TelemetrySpec> specs = [] {
    std::vector<int64_t> temperatures(20, 0), modes(20, 0);

    std::fill_n(temperatures.begin(), 16, 25);

    std::fill_n(modes.begin(), 16, 1);

    std::vector<int64_t> cells(15, 0);

    std::fill_n(cells.begin(), 8, 4200);

    return std::vector<TelemetrySpec>{
      {rclcpp::Parameter("bms_state.soc", 100), 0, 100},
      {rclcpp::Parameter("bms_state.current", int64_t{0}), INT32_MIN, INT32_MAX},
      {rclcpp::Parameter("bms_state.cell_vol", cells), 0, UINT16_MAX, 15},
      {rclcpp::Parameter("bms_state.bq_ntc", std::vector<int64_t>{25, 25}), INT8_MIN, INT8_MAX, 2},
      {rclcpp::Parameter("bms_state.mcu_ntc", std::vector<int64_t>{25, 25}), INT8_MIN, INT8_MAX, 2},
      {rclcpp::Parameter("bms_state.cycle", 0), 0, UINT16_MAX},
      {rclcpp::Parameter("bms_state.version_high", 0), 0, UINT8_MAX},
      {rclcpp::Parameter("bms_state.version_low", 0), 0, UINT8_MAX},
      {rclcpp::Parameter("bms_state.status", 0), 0, UINT8_MAX},
      {rclcpp::Parameter("power_v", 33.6), 0, 0},
      {rclcpp::Parameter("power_a", 0.0), 0, 0},
      {rclcpp::Parameter("adc_reel", 0.0), 0, 0},
      {rclcpp::Parameter("temperature_ntc1", 25), INT8_MIN, INT8_MAX},
      {rclcpp::Parameter("temperature_ntc2", 25), INT8_MIN, INT8_MAX},
      {rclcpp::Parameter("bit_flag", 0), 0, UINT8_MAX},
      {rclcpp::Parameter("motor_state.temperature", temperatures), INT8_MIN, INT8_MAX, 20},
      {rclcpp::Parameter("motor_state.mode", modes), 0, UINT8_MAX, 20},
      {rclcpp::Parameter("motor_state.lost", std::vector<int64_t>(20, 0)), 0, UINT32_MAX, 20},
      {rclcpp::Parameter("foot_force", std::vector<int64_t>(4, 0)), INT16_MIN, INT16_MAX, 4},
      {rclcpp::Parameter("foot_force_est", std::vector<int64_t>(4, 0)), INT16_MIN, INT16_MAX, 4},
      {rclcpp::Parameter("fan_frequency", std::vector<int64_t>(4, 0)), 0, UINT16_MAX, 4}};
  }();
  return specs;
}

// Validate a runtime telemetry parameter against its native type and bounds.
std::string validate_telemetry(const rclcpp::Parameter& parameter)
{
  for (const auto& spec : telemetry_specs()) {
    if (spec.initial.get_name() != parameter.get_name()) {
      continue;
    }

    if (parameter.get_type() != spec.initial.get_type()) {
      return "Incorrect parameter type";
    }

    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      if (!native_float(parameter.as_double())) {
        return "Value must be finite and fit float32";
      }

      return "";
    }

    std::vector<int64_t> values;

    if (spec.length) {
      values = parameter.as_integer_array();
    } else {
      values = {parameter.as_int()};
    }

    if (spec.length && values.size() != spec.length) {
      return "Incorrect array length";
    }

    for (size_t i = 0; i < values.size(); ++i) {
      if (values[i] < spec.minimum || values[i] > spec.maximum) {
        return "Value outside native bounds";
      }

      if (parameter.get_name().find("motor_state.") == 0 && i >= 16 && values[i] != 0) {
        return "Unused motor slots 16-19 must remain zero";
      }
    }

    return "";
  }

  return "Unknown telemetry parameter";
}

// Assemble the parameter-controlled fields of a native LowState message.
unitree_go::msg::LowState telemetry_message(const std::vector<rclcpp::Parameter>& values)
{
  std::map<std::string, rclcpp::Parameter> parameters;

  for (const auto& value : values) {
    parameters.emplace(value.get_name(), value);
  }

  unitree_go::msg::LowState message;
  auto& bms = message.bms_state;

  bms.soc = parameters.at("bms_state.soc").as_int();
  bms.current = parameters.at("bms_state.current").as_int();
  bms.cycle = parameters.at("bms_state.cycle").as_int();
  bms.version_high = parameters.at("bms_state.version_high").as_int();
  bms.version_low = parameters.at("bms_state.version_low").as_int();
  bms.status = parameters.at("bms_state.status").as_int();

  copy_integer_array(parameters, "bms_state.cell_vol", bms.cell_vol);
  copy_integer_array(parameters, "bms_state.bq_ntc", bms.bq_ntc);
  copy_integer_array(parameters, "bms_state.mcu_ntc", bms.mcu_ntc);

  message.power_v = parameters.at("power_v").as_double();
  message.power_a = parameters.at("power_a").as_double();
  message.adc_reel = parameters.at("adc_reel").as_double();
  message.temperature_ntc1 = parameters.at("temperature_ntc1").as_int();
  message.temperature_ntc2 = parameters.at("temperature_ntc2").as_int();
  message.bit_flag = parameters.at("bit_flag").as_int();

  copy_integer_array(parameters, "foot_force", message.foot_force);
  copy_integer_array(parameters, "foot_force_est", message.foot_force_est);
  copy_integer_array(parameters, "fan_frequency", message.fan_frequency);

  for (size_t i = 0; i < 20; ++i) {
    // Apply configurable values to active and reserved motor slots alike.
    message.motor_state[i].temperature =
      parameters.at("motor_state.temperature").as_integer_array()[i];

    message.motor_state[i].mode = parameters.at("motor_state.mode").as_integer_array()[i];
    message.motor_state[i].lost = parameters.at("motor_state.lost").as_integer_array()[i];
  }

  return message;
}

// Validate finite native IMU values and the quaternion norm.
bool valid_native_imu(const unitree_go::msg::IMUState& msg)
{
  double norm = 0;

  for (float value : msg.quaternion) {
    if (!std::isfinite(value)) {
      return false;
    }

    norm += value * value;
  }

  for (const auto& values : {msg.rpy, msg.gyroscope, msg.accelerometer}) {
    for (float value : values) {
      if (!std::isfinite(value)) {
        return false;
      }
    }
  }

  return std::abs(norm - 1) < 0.01;
}
} // namespace go2w_gazebo::utils
