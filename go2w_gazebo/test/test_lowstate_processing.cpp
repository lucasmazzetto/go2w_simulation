#include <gtest/gtest.h>

#include "go2w_gazebo/joint_feedback.hpp"

using namespace go2w_gazebo;
using namespace go2w_gazebo::utils;

sensor_msgs::msg::JointState joints()
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp.sec = 1;
  for (int i = 15; i >= 0; --i) {
    msg.name.push_back(MOTOR_JOINTS[i]);
    msg.position.push_back(i + 0.125);
    msg.velocity.push_back(-i - 0.25);
    msg.effort.push_back(i + 0.375);
  }
  return msg;
}

TEST(LowState, AllMotorsAreMappedByName)
{
  JointFeedback feedback;
  ASSERT_TRUE(feedback.update(joints()));
  for (size_t i = 0; i < 16; ++i) {
    const auto& m = feedback.motors[i];
    EXPECT_FLOAT_EQ(m.q, i + 0.125);
    EXPECT_FLOAT_EQ(m.dq, -static_cast<float>(i) - 0.25);
    EXPECT_FLOAT_EQ(m.tau_est, i + 0.375);
    EXPECT_FLOAT_EQ(m.q_raw, m.q);
    EXPECT_FLOAT_EQ(m.dq_raw, m.dq);
    EXPECT_FLOAT_EQ(m.ddq, 0);
    EXPECT_FLOAT_EQ(m.ddq_raw, 0);
  }
  for (size_t i = 16; i < 20; ++i) {
    EXPECT_EQ(feedback.motors[i], unitree_go::msg::MotorState{});
  }
}

TEST(LowState, DerivativesUseSimulationTimeAndReset)
{
  JointFeedback feedback;
  auto msg = joints();
  ASSERT_TRUE(feedback.update(msg));
  msg.header.stamp.nanosec = 500000000;
  for (auto& v : msg.velocity) {
    v += 2;
  }
  ASSERT_TRUE(feedback.update(msg));
  for (size_t i = 0; i < 16; ++i) {
    EXPECT_FLOAT_EQ(feedback.motors[i].ddq, 4);
    EXPECT_FLOAT_EQ(feedback.motors[i].ddq_raw, 4);
  }
  msg.header.stamp.sec = 0;
  ASSERT_TRUE(feedback.update(msg));
  EXPECT_FLOAT_EQ(feedback.motors[0].ddq, 0);
  ASSERT_TRUE(feedback.update(msg));
  EXPECT_FLOAT_EQ(feedback.motors[0].ddq, 0);
  feedback.reset();
  EXPECT_FALSE(feedback.valid);
  ASSERT_TRUE(feedback.update(msg));
  EXPECT_FLOAT_EQ(feedback.motors[0].ddq, 0);
}

TEST(LowState, RejectMalformedSamplesWithoutChangingState)
{
  JointFeedback feedback;
  ASSERT_TRUE(feedback.update(joints()));
  const auto previous = feedback.motors;
  auto msg = joints();
  msg.effort.clear();
  EXPECT_FALSE(feedback.update(msg));
  msg = joints();
  msg.name[0] = msg.name[1];
  EXPECT_FALSE(feedback.update(msg));
  msg = joints();
  msg.name[0] = "missing";
  EXPECT_FALSE(feedback.update(msg));
  msg = joints();
  msg.position[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(feedback.update(msg));
  msg = joints();
  msg.velocity[0] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(feedback.update(msg));
  msg = joints();
  msg.effort[0] = std::numeric_limits<double>::max();
  EXPECT_FALSE(feedback.update(msg));
  msg = joints();
  msg.header.stamp.nanosec = 1000000000;
  EXPECT_FALSE(feedback.update(msg));
  msg = joints();
  msg.header.stamp.sec = -1;
  EXPECT_FALSE(feedback.update(msg));
  EXPECT_EQ(feedback.motors, previous);
}

TEST(LowState, FreshnessIsBoundedBySimulationTime)
{
  JointFeedback feedback;
  EXPECT_FALSE(feedback.fresh(1000000000));
  ASSERT_TRUE(feedback.update(joints()));
  EXPECT_TRUE(feedback.fresh(1000000000));
  EXPECT_TRUE(feedback.fresh(2000000000));
  EXPECT_FALSE(feedback.fresh(2000000001));
  EXPECT_FALSE(feedback.fresh(999999999));
}

TEST(LowState, DefaultsAndNativeMetadata)
{
  std::vector<rclcpp::Parameter> values;
  for (const auto& spec : telemetry_specs()) {
    EXPECT_TRUE(validate_telemetry(spec.initial).empty()) << spec.initial.get_name();
    values.push_back(spec.initial);
  }
  const auto msg = telemetry_message(values);
  EXPECT_EQ(msg.bms_state.soc, 100);
  EXPECT_EQ(msg.bms_state.current, 0);
  EXPECT_FLOAT_EQ(msg.power_v, 33.6f);
  EXPECT_FLOAT_EQ(msg.power_a, 0);
  EXPECT_EQ(msg.temperature_ntc1, 25);
  EXPECT_EQ(msg.bms_state.bq_ntc[0], 25);
  for (size_t i = 0; i < 15; ++i) {
    int64_t expected_cell_voltage = 0;

    if (i < 8) {
      expected_cell_voltage = 4200;
    }

    EXPECT_EQ(msg.bms_state.cell_vol[i], expected_cell_voltage);
  }
  for (size_t i = 0; i < 20; ++i) {
    int64_t expected_temperature = 0;
    int64_t expected_mode = 0;

    if (i < 16) {
      expected_temperature = 25;
      expected_mode = 1;
    }

    EXPECT_EQ(msg.motor_state[i].temperature, expected_temperature);
    EXPECT_EQ(msg.motor_state[i].mode, expected_mode);
  }
  EXPECT_EQ(msg.head[0], 0);
  EXPECT_EQ(msg.crc, 0u);
  EXPECT_EQ(msg.reserve, 0u);
  EXPECT_EQ(msg.wireless_remote[0], 0);
}

TEST(LowState, ParameterTypesBoundsAndLengths)
{
  for (const auto& spec : telemetry_specs()) {
    const auto name = spec.initial.get_name();
    EXPECT_FALSE(validate_telemetry(rclcpp::Parameter(name, "wrong")).empty());
    if (spec.length) {
      EXPECT_FALSE(validate_telemetry(rclcpp::Parameter(name, std::vector<int64_t>{})).empty());
      auto values = spec.initial.as_integer_array();
      values[0] = spec.maximum + 1;
      EXPECT_FALSE(validate_telemetry(rclcpp::Parameter(name, values)).empty());
      values[0] = spec.minimum - 1;
      EXPECT_FALSE(validate_telemetry(rclcpp::Parameter(name, values)).empty());
      if (name.find("motor_state.") == 0) {
        values = spec.initial.as_integer_array();
        values[19] = 1;
        EXPECT_FALSE(validate_telemetry(rclcpp::Parameter(name, values)).empty());
      }
    } else if (spec.initial.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      EXPECT_FALSE(validate_telemetry(rclcpp::Parameter(name, spec.maximum + 1)).empty());
      EXPECT_FALSE(validate_telemetry(rclcpp::Parameter(name, spec.minimum - 1)).empty());
    } else {
      EXPECT_FALSE(validate_telemetry(rclcpp::Parameter(name, INFINITY)).empty());
      EXPECT_FALSE(validate_telemetry(rclcpp::Parameter(name, NAN)).empty());
    }
  }
}

TEST(LowState, ParametersRemainIndependent)
{
  std::vector<rclcpp::Parameter> values;
  for (const auto& spec : telemetry_specs()) {
    if (spec.initial.get_name() == "bms_state.soc") {
      values.push_back(rclcpp::Parameter("bms_state.soc", 30));
    } else {
      values.push_back(spec.initial);
    }
  }
  const auto msg = telemetry_message(values);
  EXPECT_EQ(msg.bms_state.soc, 30);
  EXPECT_EQ(msg.bms_state.cell_vol[0], 4200);
  EXPECT_FLOAT_EQ(msg.power_v, 33.6f);

  // Overriding live values must not change the shared defaults used by other nodes.
  std::vector<rclcpp::Parameter> defaults;
  for (const auto& spec : telemetry_specs()) {
    defaults.push_back(spec.initial);
  }
  const auto default_msg = telemetry_message(defaults);
  EXPECT_EQ(default_msg.bms_state.soc, 100);
  EXPECT_EQ(default_msg.bms_state.cell_vol[0], 4200);
  EXPECT_FLOAT_EQ(default_msg.power_v, 33.6f);
}
