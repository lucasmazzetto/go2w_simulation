#include <gtest/gtest.h>
#include <limits>
#include "go2w_gazebo/utils.hpp"

using namespace go2w_gazebo;
using namespace go2w_gazebo::utils;

TEST(Imu, NativeOrderUnitsAndYaw)
{
  sensor_msgs::msg::Imu msg;
  msg.orientation.w = msg.orientation.z = std::sqrt(0.5);
  msg.linear_acceleration.z = 9.81;
  msg.angular_velocity.z = 0.4;
  ASSERT_TRUE(valid_imu(msg));
  const auto native = native_imu(msg, 25);
  EXPECT_NEAR(native.quaternion[0], std::sqrt(0.5), 1e-6);
  EXPECT_EQ(native.quaternion[1], 0);
  EXPECT_NEAR(native.quaternion[3], std::sqrt(0.5), 1e-6);
  EXPECT_NEAR(native.rpy[2], std::acos(-1) / 2, 1e-6);
  EXPECT_NEAR(native.accelerometer[2], 9.81, 1e-6);
  EXPECT_NEAR(native.gyroscope[2], 0.4, 1e-6);
  EXPECT_EQ(native.temperature, 25);
  msg.orientation.w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(valid_imu(msg));
}

TEST(Imu, CovarianceAndInvalidQuaternion)
{
  sensor_msgs::msg::Imu msg;
  msg.orientation.w = 0;
  EXPECT_FALSE(valid_imu(msg));
  set_covariances(msg, 0.02, 0.1);
  EXPECT_EQ(msg.orientation_covariance[0], 0);
  EXPECT_DOUBLE_EQ(msg.angular_velocity_covariance[4], 0.0004);
  EXPECT_DOUBLE_EQ(msg.linear_acceleration_covariance[8], 0.1 * 0.1);
  EXPECT_EQ(msg.linear_acceleration_covariance[1], 0);
}

sensor_msgs::msg::PointCloud2 input_cloud()
{
  sensor_msgs::msg::PointCloud2 cloud;
  using Field = sensor_msgs::msg::PointField;
  for (const auto& pair : std::array<std::pair<std::string, uint32_t>, 5>{
         {{"x", 0}, {"y", 4}, {"z", 8}, {"intensity", 12}, {"ring", 16}}}) {
    Field field;
    field.name = pair.first;
    field.offset = pair.second;

    if (field.name == "ring") {
      field.datatype = Field::UINT16;
    } else {
      field.datatype = Field::FLOAT32;
    }

    field.count = 1;
    cloud.fields.push_back(field);
  }
  cloud.width = 2;
  cloud.height = 2;
  cloud.point_step = 20;
  cloud.row_step = 48; // row padding must be respected
  cloud.data.resize(96, 0);
  cloud.header.stamp.sec = 42;
  for (size_t row = 0; row < 2; ++row) {
    for (size_t col = 0; col < 2; ++col) {
      auto* point = cloud.data.data() + row * 48 + col * 20;
      write_float(point, 1 + row * 2 + col);
      write_float(point + 12, 7);
      point[16] = row;
    }
  }
  return cloud;
}

TEST(Cloud, PaddingFilteringLayoutAndStamp)
{
  auto source = input_cloud();
  write_float(source.data.data() + 20, std::numeric_limits<float>::infinity());
  const auto output = format_cloud(source, "radar", 0.05, 3.5);
  ASSERT_EQ(output.width, 2u);
  EXPECT_EQ(output.height, 1u);
  EXPECT_EQ(output.header.stamp.sec, 42);
  EXPECT_EQ(output.header.frame_id, "radar");
  EXPECT_EQ(output.row_step, 48u);
  EXPECT_FLOAT_EQ(read_float(output.data.data(), false), 1);
  EXPECT_FLOAT_EQ(read_float(output.data.data() + 24, false), 3);
  EXPECT_FLOAT_EQ(read_float(output.data.data() + 24 + 20, false), 0);
  EXPECT_EQ(output.data[24 + 16], 1u);
  EXPECT_EQ(output.fields[4].datatype, sensor_msgs::msg::PointField::UINT16);
}

TEST(Cloud, BigEndianAndEmptyScans)
{
  auto source = input_cloud();
  for (size_t row = 0; row < 2; ++row) {
    for (size_t col = 0; col < 2; ++col) {
      auto* point = source.data.data() + row * 48 + col * 20;
      for (size_t field = 0; field < 4; ++field) {
        std::reverse(point + field * 4, point + field * 4 + 4);
      }
      std::swap(point[16], point[17]);
    }
  }
  source.is_bigendian = true;
  auto output = format_cloud(source, "radar", 0.05, 20);
  ASSERT_EQ(output.width, 4u);
  EXPECT_FLOAT_EQ(read_float(output.data.data() + 72, false), 4);
  EXPECT_EQ(output.data[72 + 16], 1u);
  output = format_cloud(source, "radar", 10, 20);
  EXPECT_EQ(output.width, 0u);
  EXPECT_EQ(output.height, 1u);
  EXPECT_TRUE(output.data.empty());
  EXPECT_EQ(output.fields.size(), 6u);
}

TEST(Cloud, RejectsTruncatedDataAndWrongFields)
{
  auto source = input_cloud();
  source.data.pop_back();
  EXPECT_THROW(format_cloud(source, "radar", 0.05, 20), std::runtime_error);
  source = input_cloud();
  source.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT64;
  EXPECT_THROW(format_cloud(source, "radar", 0.05, 20), std::runtime_error);
}
