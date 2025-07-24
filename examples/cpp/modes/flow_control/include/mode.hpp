/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Author: Your Name
 ****************************************************************************/
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>
#include <px4_msgs/msg/vehicle_optical_flow.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <cmath>
#include <algorithm>

// Define a unique name for the mode
static const std::string kModeName = "Flow Control";

class FlightModeTest : public px4_ros2::ModeBase
{
public:
  explicit FlightModeTest(rclcpp::Node & node)
  : ModeBase(node, kModeName)
  {
    // Required setpoint type for mode registration
    _attitude_setpoint = std::make_shared<px4_ros2::AttitudeSetpointType>(*this);

    // Define a QoS profile compatible with PX4 sensor data
    rclcpp::QoS qos_profile(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    // Create subscribers for sensor data
    _distance_sensor_sub =
      node.create_subscription<px4_msgs::msg::DistanceSensor>(
      "/fmu/out/distance_sensor", qos_profile,
      std::bind(&FlightModeTest::distanceSensorCallback, this, std::placeholders::_1));

    _optical_flow_sub =
      node.create_subscription<px4_msgs::msg::VehicleOpticalFlow>(
      "/fmu/out/vehicle_optical_flow", qos_profile,
      std::bind(&FlightModeTest::opticalFlowCallback, this, std::placeholders::_1));
  }

  void onActivate() override
  {
    // Reset state when the mode is activated
    _is_first_distance_message = true;
    _vertical_rate_mps = 0.f;
    _current_distance_m = -1.f;
    _flow_x_rad_s = 0.f;
    _flow_y_rad_s = 0.f;

    // Reset PI controller integrals
    _vertical_integral = 0.f;
    _flow_x_integral = 0.f;
    _flow_y_integral = 0.f;

    // Reset time for sinusoidal command
    _total_time_s = 0.f;
    _yaw = 0.f; // Initialize yaw

    RCLCPP_INFO(node().get_logger(), "Flow Control mode activated.");
  }

  void onDeactivate() override
  {
     RCLCPP_INFO(node().get_logger(), "Flow Control mode deactivated.");
  }

  void updateSetpoint(float dt_s) override
  {
    // ============== DEBUG: SINUSOIDAL THRUST COMMAND (Commented out) ==============
    /*
    _total_time_s += dt_s;

    // Parameters for the sine wave
    const float amplitude = 0.15f; // 15% thrust variation
    const float frequency_hz = 0.5f; // 0.5 cycles per second
    const float angular_frequency = 2.f * M_PI * frequency_hz;

    // Calculate the sinusoidal thrust command (0 to 1)
    const float thrust_value = kHoverThrust + amplitude * sinf(angular_frequency * _total_time_s);

    // Keep pitch and roll commands at zero for this test
    const float pitch_cmd = 0.f;
    const float roll_cmd = 0.f;

    // Construct the attitude quaternion
    const Eigen::Quaternionf attitude_q = px4_ros2::eulerRpyToQuaternion(roll_cmd, pitch_cmd, _yaw);

    // Construct the thrust vector. For multicopters, thrust is along the negative Z body axis.
    const Eigen::Vector3f thrust_sp{0.f, 0.f, -thrust_value};

    RCLCPP_INFO(
      node().get_logger(), "DEBUG: Sending Thrust Value: %.3f", thrust_value);

    // Send the final attitude command to the flight controller
    _attitude_setpoint->update(attitude_q, thrust_sp);
    */


    // ============== PI CONTROLLER LOGIC ==============
    // Run PI controllers to get control outputs
    const float thrust_cmd_pi = calculateVerticalThrust(dt_s);
    const float pitch_cmd_pi = calculatePitchCommand(dt_s);
    const float roll_cmd_pi = calculateRollCommand(dt_s);

    // Construct the attitude quaternion
    const Eigen::Quaternionf attitude_q_pi = px4_ros2::eulerRpyToQuaternion(roll_cmd_pi, pitch_cmd_pi, _yaw);

    // Construct the thrust vector
    const Eigen::Vector3f thrust_sp_pi{0.f, 0.f, -thrust_cmd_pi};

    // Send the final attitude command to the flight controller
    _attitude_setpoint->update(attitude_q_pi, thrust_sp_pi);
  }

private:
  // ============== PI Controller Constants (TUNING REQUIRED) ==============
  const float kP_vertical = 0.2f;
  const float kI_vertical = 0.05f;
  const float kHoverThrust = 0.5f; // Nominal thrust for hovering

  const float kP_flow = 0.1f;
  const float kI_flow = 0.02f;

  // Safety limits
  const float kMaxPitchRoll = px4_ros2::degToRad(15.f);
  const float kMinThrust = 0.1f;
  const float kMaxThrust = 0.8f;
  const float kLandingThrust = 0.3f;
  const float kSafetyHeightMax = 2.0f;

  // ============== Controller Implementations ==============
  float calculateVerticalThrust(float dt_s)
  {
    if (_current_distance_m > kSafetyHeightMax) {
      RCLCPP_WARN_THROTTLE(
        node().get_logger(), *node().get_clock(), 2000,
        "Above safety height (%.2f m)! Commanding descent.", _current_distance_m);
      return kLandingThrust;
    }
    const float error = 0.f - _vertical_rate_mps;
    _vertical_integral += error * dt_s;
    _vertical_integral = std::clamp(_vertical_integral, -0.5f, 0.5f);
    float thrust = kHoverThrust + kP_vertical * error + kI_vertical * _vertical_integral;
    return std::clamp(thrust, kMinThrust, kMaxThrust);
  }

  float calculatePitchCommand(float dt_s)
  {
    const float error = 0.f - _flow_y_rad_s;
    _flow_y_integral += error * dt_s;
    _flow_y_integral = std::clamp(_flow_y_integral, -0.5f, 0.5f);
    float pitch = kP_flow * error + kI_flow * _flow_y_integral;
    return std::clamp(pitch, -kMaxPitchRoll, kMaxPitchRoll);
  }

  float calculateRollCommand(float dt_s)
  {
    const float error = 0.f - _flow_x_rad_s;
    _flow_x_integral += error * dt_s;
    _flow_x_integral = std::clamp(_flow_x_integral, -0.5f, 0.5f);
    float roll = -(kP_flow * error + kI_flow * _flow_x_integral);
    return std::clamp(roll, -kMaxPitchRoll, kMaxPitchRoll);
  }

  // ============== Sensor Data Callbacks ==============
  void distanceSensorCallback(const px4_msgs::msg::DistanceSensor::SharedPtr msg)
  {
    _current_distance_m = msg->current_distance;
    if (_is_first_distance_message) {
      if (std::isfinite(msg->current_distance)) {
        _previous_timestamp_us = msg->timestamp;
        _previous_distance_m = msg->current_distance;
        _is_first_distance_message = false;
        RCLCPP_INFO(
          node().get_logger(), "First valid distance received: %.3f m",
          msg->current_distance);
      }
      return;
    }
    if (!std::isfinite(msg->current_distance)) { return; }
    const double delta_time_s = (msg->timestamp - _previous_timestamp_us) / 1e6;
    if (delta_time_s <= 0.0) {
      _previous_timestamp_us = msg->timestamp;
      return;
    }
    const double delta_distance_m = msg->current_distance - _previous_distance_m;
    _vertical_rate_mps = delta_distance_m / delta_time_s;
    _previous_timestamp_us = msg->timestamp;
    _previous_distance_m = msg->current_distance;
  }

  void opticalFlowCallback(const px4_msgs::msg::VehicleOpticalFlow::SharedPtr msg)
  {
    if (msg->quality > 0) {
      _flow_x_rad_s = msg->pixel_flow[0];
      _flow_y_rad_s = msg->pixel_flow[1];
    }
  }

  // Setpoint type
  std::shared_ptr<px4_ros2::AttitudeSetpointType> _attitude_setpoint;

  // Subscribers
  rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr _distance_sensor_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleOpticalFlow>::SharedPtr _optical_flow_sub;

  // State variables
  uint64_t _previous_timestamp_us{0};
  float _previous_distance_m{0.0f};
  bool _is_first_distance_message{true};
  float _total_time_s{0.f};
  float _yaw{0.f};

  // Sensor values
  float _vertical_rate_mps{0.f};
  float _current_distance_m{-1.f};
  float _flow_x_rad_s{0.f};
  float _flow_y_rad_s{0.f};

  // PI controller integrals
  float _vertical_integral{0.f};
  float _flow_x_integral{0.f};
  float _flow_y_integral{0.f};
};
