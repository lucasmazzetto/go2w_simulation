#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <rclcpp/parameter.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <stdexcept>
#include <string>
#include <unitree_go/msg/imu_state.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <vector>

namespace go2w_gazebo::utils
{
/**
 * @brief Checks that an IMU message has finite values and a valid quaternion.
 * @param msg IMU message to validate.
 * @return True when all required values are valid.
 */
bool valid_imu(const sensor_msgs::msg::Imu& msg);

/**
 * @brief Converts a ROS IMU message to the native Unitree IMU message.
 * @param msg ROS IMU message to convert.
 * @param temperature Temperature value assigned to the native message.
 * @return Converted native IMU state.
 */
unitree_go::msg::IMUState native_imu(const sensor_msgs::msg::Imu& msg, int temperature);

/**
 * @brief Sets diagonal gyro and accelerometer covariance values.
 * @param msg IMU message whose covariance fields are updated.
 * @param gyro Diagonal angular-velocity covariance value.
 * @param acceleration Diagonal linear-acceleration covariance value.
 */
void set_covariances(sensor_msgs::msg::Imu& msg, double gyro, double acceleration);

/**
 * @brief Reads an unsigned integer from a point-cloud byte sequence.
 * @param data Start of the byte sequence.
 * @param length Number of bytes to read.
 * @param big_endian Whether the byte sequence uses big-endian ordering.
 * @return Decoded unsigned integer.
 */
uint32_t read_bits(const uint8_t* data, size_t length, bool big_endian);

/**
 * @brief Reads a 32-bit floating-point value from point-cloud data.
 * @param data Start of the four-byte floating-point value.
 * @param big_endian Whether the byte sequence uses big-endian ordering.
 * @return Decoded floating-point value.
 */
float read_float(const uint8_t* data, bool big_endian);

/**
 * @brief Writes a little-endian 32-bit floating-point value.
 * @param data Destination for the four-byte floating-point value.
 * @param value Floating-point value to write.
 */
void write_float(uint8_t* data, float value);

/**
 * @brief Finds and validates a named point-cloud field.
 * @param cloud Point cloud whose fields are searched.
 * @param name Field name to find.
 * @param type Expected point-field datatype.
 * @param size Required field size in bytes.
 * @return Byte offset of the validated field.
 */
uint32_t field_offset(const sensor_msgs::msg::PointCloud2& cloud, const std::string& name,
                      uint8_t type, uint32_t size);

/**
 * @brief Converts a sensor cloud to the public Go2-W point-cloud layout.
 * @param source Input point cloud.
 * @param frame Frame assigned to the converted cloud.
 * @param range_min Minimum accepted point range.
 * @param range_max Maximum accepted point range.
 * @return Converted point cloud.
 */
sensor_msgs::msg::PointCloud2 format_cloud(const sensor_msgs::msg::PointCloud2& source,
                                           const std::string& frame, double range_min,
                                           double range_max);

/**
 * @brief Validates the layout and metadata of a camera image.
 * @param image Camera image to validate.
 * @param frame Expected image frame.
 * @param width Expected image width in pixels.
 * @param height Expected image height in pixels.
 * @return True when the image metadata and layout are valid.
 */
bool valid_camera_image(const sensor_msgs::msg::Image& image, const std::string& frame,
                        uint32_t width, uint32_t height);

/**
 * @brief Validates camera calibration metadata against configured intrinsics.
 * @param info Camera calibration message to validate.
 * @param frame Expected optical frame.
 * @param width Expected image width in pixels.
 * @param height Expected image height in pixels.
 * @param intrinsics Expected intrinsic parameters in fx, fy, cx, cy order.
 * @return True when the calibration metadata matches the configuration.
 */
bool valid_camera_info(const sensor_msgs::msg::CameraInfo& info, const std::string& frame,
                       uint32_t width, uint32_t height, const std::vector<double>& intrinsics);

/**
 * @brief Compares finite floating-point values within the camera tolerance.
 * @param first First value to compare.
 * @param second Second value to compare.
 * @return True when both values are finite and sufficiently close.
 */
bool close_enough(double first, double second);

/**
 * @brief Checks whether a floating-point value is exactly zero.
 * @param value Value to check.
 * @return True when the value is exactly zero.
 */
bool is_zero(double value);

/**
 * @brief Maps native Unitree motor indices to simulation joint names.
 */
inline const std::array<std::string, 16> MOTOR_JOINTS = {
  "FR_hip_joint",   "FR_thigh_joint", "FR_calf_joint",  "FL_hip_joint",
  "FL_thigh_joint", "FL_calf_joint",  "RR_hip_joint",   "RR_thigh_joint",
  "RR_calf_joint",  "RL_hip_joint",   "RL_thigh_joint", "RL_calf_joint",
  "FR_foot_joint",  "FL_foot_joint",  "RR_foot_joint",  "RL_foot_joint"};

/**
 * @brief Checks whether a value is finite and representable as a float32.
 * @param value Value to validate.
 * @return True when the value can be represented safely as a float32.
 */
bool native_float(double value);

/**
 * @brief Describes validation rules and the default for one LowState parameter.
 */
struct TelemetrySpec
{
  /**
   * @var initial
   * Default value for the parameter.
   */
  rclcpp::Parameter initial;

  /**
   * @var minimum
   * Smallest accepted integer value.
   */
  int64_t minimum;

  /**
   * @var maximum
   * Largest accepted integer value.
   */
  int64_t maximum;

  /**
   * @var length
   * Required array length, or zero for scalar parameters.
   */
  size_t length{0};
};

/**
 * @brief Returns all configurable LowState telemetry defaults and bounds.
 * @return Shared immutable specifications in parameter declaration order,
 *         valid for the lifetime of the process.
 */
const std::vector<TelemetrySpec>& telemetry_specs();

/**
 * @brief Validates one configurable LowState parameter.
 * @param parameter Parameter value to validate.
 * @return An empty string when valid, otherwise a validation error.
 */
std::string validate_telemetry(const rclcpp::Parameter& parameter);

/**
 * @brief Builds the parameter-controlled portion of a native LowState message.
 * @param values Configured LowState parameter values in specification order.
 * @return Native LowState message containing the configured telemetry.
 */
unitree_go::msg::LowState telemetry_message(const std::vector<rclcpp::Parameter>& values);

/**
 * @brief Validates a native Unitree IMU state.
 * @param msg Native IMU state to validate.
 * @return True when all required values are finite and valid.
 */
bool valid_native_imu(const unitree_go::msg::IMUState& msg);

/**
 * @brief Copies an integer parameter array into a native message array.
 * @tparam Destination Destination array type.
 * @param parameters Parameter map to search.
 * @param key Name of the requested array parameter.
 * @param destination Destination array to populate.
 */
template <typename Destination>
void copy_integer_array(const std::map<std::string, rclcpp::Parameter>& parameters,
                        const std::string& key, Destination& destination)
{
  const auto& source = parameters.at(key).as_integer_array();
  std::copy(source.begin(), source.end(), destination.begin());
}
} // namespace go2w_gazebo::utils
