#include <gtest/gtest.h>

#include <limits>

#include "go2w_gazebo/utils.hpp"

namespace
{
const std::vector<double> k = {864.39938, 0, 639.19798, 0, 863.73849, 373.28118, 0, 0, 1};

sensor_msgs::msg::CameraInfo camera_info()
{
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = "front_camera_optical";
  info.header.stamp.sec = 1;
  info.width = 1280;
  info.height = 720;
  std::copy(k.begin(), k.end(), info.k.begin());
  info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  info.p = {k[0], 0, k[2], 0, 0, k[4], k[5], 0, 0, 0, 1, 0};
  info.d.assign(5, 0);
  info.distortion_model = "plumb_bob";
  return info;
}
} // namespace

TEST(CameraValidation, RejectsBadImages)
{
  sensor_msgs::msg::Image image;
  image.header.frame_id = "front_camera_optical";
  image.header.stamp.sec = 1;
  image.width = 1280;
  image.height = 720;
  image.encoding = "rgb8";
  image.step = image.width * 3;
  image.data.resize(image.step * image.height);
  EXPECT_TRUE(go2w_gazebo::utils::valid_camera_image(image, "front_camera_optical", 1280, 720));
  image.data.pop_back();
  EXPECT_FALSE(go2w_gazebo::utils::valid_camera_image(image, "front_camera_optical", 1280, 720));
  image.data.push_back(0);
  image.encoding = "bgr8";
  EXPECT_FALSE(go2w_gazebo::utils::valid_camera_image(image, "front_camera_optical", 1280, 720));
  image.encoding = "rgb8";
  EXPECT_FALSE(go2w_gazebo::utils::valid_camera_image(image, "wrong_frame", 1280, 720));
  EXPECT_FALSE(go2w_gazebo::utils::valid_camera_image(image, "front_camera_optical", 640, 360));
  image.header.stamp.sec = 0;
  EXPECT_FALSE(go2w_gazebo::utils::valid_camera_image(image, "front_camera_optical", 1280, 720));
}

TEST(CameraValidation, RequiresConsistentCalibration)
{
  auto info = camera_info();
  EXPECT_TRUE(go2w_gazebo::utils::valid_camera_info(info, "front_camera_optical", 1280, 720, k));
  info.k[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(go2w_gazebo::utils::valid_camera_info(info, "front_camera_optical", 1280, 720, k));
  info = camera_info();
  info.d[0] = -0.3;
  EXPECT_FALSE(go2w_gazebo::utils::valid_camera_info(info, "front_camera_optical", 1280, 720, k));
  info = camera_info();
  info.p[0] *= 2;
  EXPECT_FALSE(go2w_gazebo::utils::valid_camera_info(info, "front_camera_optical", 1280, 720, k));
  info = camera_info();
  info.r.fill(0);
  EXPECT_FALSE(go2w_gazebo::utils::valid_camera_info(info, "front_camera_optical", 1280, 720, k));
  info = camera_info();
  EXPECT_FALSE(go2w_gazebo::utils::valid_camera_info(info, "front_camera_optical", 640, 360, k));
}
