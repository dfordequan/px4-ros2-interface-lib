/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Author: Your Name
 ****************************************************************************/
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>
#include <std_msgs/msg/float32_multi_array.hpp> // For the new custom flow sensor
#include <px4_msgs/msg/hover_thrust_estimate.hpp>   //  <-- NEW for thrust
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <cmath>
#include <algorithm>
#include <fstream>      // For file I/O (CSV logging)
#include <string>       // For string manipulation
#include <chrono>       // For timestamping files
#include <iomanip>      // For formatting timestamps

// Define a unique name for the mode
static const std::string kModeName = "Flow Control";

class FlightModeTest : public px4_ros2::ModeBase
{
public:
  explicit FlightModeTest(rclcpp::Node & node)
  : ModeBase(node, kModeName)
  {
    // A setpoint type is required for the mode to register successfully.
    _attitude_setpoint = std::make_shared<px4_ros2::AttitudeSetpointType>(*this);

    // Declare tunable parameters
    declareParameters();

    // Define a QoS profile compatible with PX4 sensor data
    rclcpp::QoS qos_profile(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    // Create subscribers for sensor data
    _distance_sensor_sub =
      node.create_subscription<px4_msgs::msg::DistanceSensor>(
      "/fmu/out/distance_sensor", qos_profile,
      std::bind(&FlightModeTest::distanceSensorCallback, this, std::placeholders::_1));

    // Subscriber for the new custom optical flow from the 'speck' node
    _custom_flow_sub =
      node.create_subscription<std_msgs::msg::Float32MultiArray>(
      "/optical_flow/estimated", 10,
      std::bind(&FlightModeTest::customFlowCallback, this, std::placeholders::_1));

    // Subscriber for hover thrust estimate (optional, if you want to use it)
    _hover_thrust_sub =
      node.create_subscription<px4_msgs::msg::HoverThrustEstimate>(
          "/fmu/out/hover_thrust_estimate",
          qos_profile,                              // or rclcpp::SensorDataQoS{}
          std::bind(&FlightModeTest::hoverThrustCallback,
                    this, std::placeholders::_1));
  }
  

  void onActivate() override
  {
    // Reset state when the mode is activated
    _current_distance_m = -1.f;
    _flow_x_rad_s = 0.f;
    _flow_y_rad_s = 0.f;
    _divergence = 0.f;

    // Reset PI controller integrals
    _vertical_integral = 0.f;
    _flow_x_integral = 0.f;
    _flow_y_integral = 0.f;

    _yaw = 0.f;

    // --- CSV Logging Setup ---
    // Create a unique filename with the current date and time
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "flow_control_log_%Y%m%d_%H%M%S.csv");
    _csv_log_file.open(ss.str());
    if (_csv_log_file.is_open()) {
      // Write the header row
      _csv_log_file << "timestamp_us,flow_x,flow_y,divergence,roll_cmd_deg,pitch_cmd_deg,thrust_cmd,current_distance_m\n";
      RCLCPP_INFO(node().get_logger(), "Opened log file: %s", ss.str().c_str());
    } else {
      RCLCPP_ERROR(node().get_logger(), "Failed to open log file!");
    }

    RCLCPP_INFO(node().get_logger(), "Flow Control mode activated. Holding yaw at: %.2f deg", px4_ros2::radToDeg(_yaw));

    // Capture the current height as the AUTO set-point (fallback to 1 m)
    if (_current_distance_m > 0.f && std::isfinite(_current_distance_m)) {
        _h_setpoint = _current_distance_m;
    } else {
        _h_setpoint = 1.0f;           // sonar not ready → default 1 m
    }

    // Clear the I-term so we start bias-free
    _vertical_integral = 0.f;

    // put this at the very END of onActivate()
    RCLCPP_WARN_ONCE(node().get_logger(),
        "PI kp=%.3f  ki=%.3f  hover=%.3f  clamp=[0.22,0.30]",
        _p_divergence, _i_divergence, _hover_thrust);
  }

  void onDeactivate() override
  {
     // --- Close the CSV file cleanly ---
    if (_csv_log_file.is_open()) {
      _csv_log_file.close();
      RCLCPP_INFO(node().get_logger(), "Closed log file.");
    }
     RCLCPP_INFO(node().get_logger(), "Flow Control mode deactivated.");
  }

  void updateSetpoint(float dt_s) override
  {
    // ============== PI CONTROLLER LOGIC (ATTITUDE SETPOINT) ==============
    // Run all PI controllers to get pitch, roll, and thrust commands
    const float thrust_cmd = calculateVerticalThrust(dt_s); 
    const float pitch_cmd_pi = calculatePitchCommand(dt_s);
    const float roll_cmd_pi = calculateRollCommand(dt_s);

    //-------FINISH NEW ADD------------

    // Construct the attitude quaternion using the latched yaw
    const Eigen::Quaternionf attitude_q_pi = px4_ros2::eulerRpyToQuaternion(roll_cmd_pi, pitch_cmd_pi, _yaw);

    // Construct the thrust vector with the calculated thrust
    const Eigen::Vector3f thrust_sp_pi{0.f, 0.f, -thrust_cmd};

    // Send the final attitude command to the flight controller
    _attitude_setpoint->update(attitude_q_pi, thrust_sp_pi);

    // --- CSV Logging ---
    if (_csv_log_file.is_open()) {
      _csv_log_file << node().get_clock()->now().nanoseconds() / 1000 << ","
                    << _flow_x_rad_s << ","
                    << _flow_y_rad_s << ","
                    << _divergence << ","
                    << px4_ros2::radToDeg(roll_cmd_pi) << ","
                    << px4_ros2::radToDeg(pitch_cmd_pi) << ","
                    << thrust_cmd     << ","
                    << _current_distance_m              // <-- new field
                    << "\n";
    }

    // log the setpoint
    RCLCPP_INFO_THROTTLE(
      node().get_logger(), *node().get_clock(), 50, // Log every 50ms
      "Setpoint -> R: %.3f, P: %.3f, T: %.3f",
      px4_ros2::radToDeg(roll_cmd_pi), px4_ros2::radToDeg(pitch_cmd_pi), thrust_cmd);
  }

private:
  // ============== PI Controller Constants (Now ROS 2 Parameters) ==============
  double _p_divergence, _i_divergence, _hover_thrust;
  double _p_flow, _i_flow;
  double _sensor_filter_alpha;
  double _flow_offset_x, _flow_offset_y;

  // CSV log file stream
  std::ofstream _csv_log_file;

  // Safety limits (still constant)
  const float kMaxPitchRoll = px4_ros2::degToRad(15.f);
  const float kLandingThrust = 0.18f;
  //const float kSafetyHeightMax = 3.0f; //2.0

  void declareParameters()
  {
      // Divergence (vertical) controller gains
      node().declare_parameter<double>("gains.divergence.p", 0.9); // 8.0, 0.05, 0.02, #better 0.08, 0.12, 0.10 best  0.9, trying 5.0
      node().declare_parameter<double>("gains.divergence.i", 0.12);// 2.0, 0.01, 0.05, #better 0.05, 0.04, 0.05 best  0.12, trying 0.08
      //node().declare_parameter<double>("gains.divergence.hover_thrust", 0.255); // Center of [0.2, 0.27] range
 

      // Flow (horizontal) controller gains
      node().declare_parameter<double>("gains.flow.p", 0.05);//0.2, better 0.08,  good: 0.05, best till now 0.05
      node().declare_parameter<double>("gains.flow.i", 0.1); //0.1, better 0.08, good: 0.05,  best till now 0.08   , 

      // Filter and limit parameters
      node().declare_parameter<double>("filters.sensor_alpha", 0.9);

      // Offsets for the new custom flow sensor
      node().declare_parameter<double>("offsets.flow_x", 0.36);
      node().declare_parameter<double>("offsets.flow_y", 0.16);

  }

  // ============== Controller Implementations ==============
  float calculateVerticalThrust(float dt_s)
  {
    // Get the latest parameter values
    node().get_parameter("gains.divergence.p", _p_divergence);
    node().get_parameter("gains.divergence.i", _i_divergence);
    //node().get_parameter("gains.divergence.hover_thrust", _hover_thrust);

    // ----- bail out until sonar valid ------------------------------------
    // if (_current_distance_m < 0.05f || !std::isfinite(_current_distance_m)) {
    //     return _hover_thrust;                 // hold nominal thrust
    // }
    if (!std::isfinite(_divergence)) {               // never arrived yet
        return _hover_thrust;                        // hold
    }

    //if (_current_distance_m > kSafetyHeightMax && _current_distance_m > 0) {
    //  RCLCPP_WARN_THROTTLE(
    //   node().get_logger(), *node().get_clock(), 2000,
    //    "Above safety height (%.2f m)! Commanding descent.", _current_distance_m);
    //  return kLandingThrust;
    //}

    // PI controller based on divergence. 
    const float error = _divergence; //1.0f - _current_distance_m; // no need for 0 - div
    //const float error = _h_setpoint - _current_distance_m;

    /* -------- 4. integral update  ----------------------------- */

    _vertical_integral += error * dt_s;
    _vertical_integral = std::clamp(_vertical_integral, -0.5f, 0.5f); 

    float thrust = _hover_thrust
                 + _p_divergence * error
                 + _i_divergence * _vertical_integral; // earlier - before(p_divergence * _divergence);

    // Clamp final thrust to your specified range [0.2, 0.30]
    return std::clamp(thrust, 0.12f, 0.30f); 
  }

  float calculatePitchCommand(float dt_s)
  {
    // Get the latest parameter values
    node().get_parameter("gains.flow.p", _p_flow);
    node().get_parameter("gains.flow.i", _i_flow);

    const float error = 0.f - _flow_y_rad_s; // Setpoint is 0, 
    _flow_y_integral += error * dt_s;
    _flow_y_integral = std::clamp(_flow_y_integral, -1.0f, 1.0f); 
    float pitch = -(_p_flow * error + _i_flow * _flow_y_integral); // put minus sign.
    return std::clamp(pitch, -kMaxPitchRoll, kMaxPitchRoll);
    }

  float calculateRollCommand(float dt_s)
  {
    // Get the latest parameter values
    node().get_parameter("gains.flow.p", _p_flow);
    node().get_parameter("gains.flow.i", _i_flow);

    const float error = 0.f - _flow_x_rad_s; // Setpoint is 0
    _flow_x_integral += error * dt_s;
    _flow_x_integral = std::clamp(_flow_x_integral, -1.0f, 1.0f);
    float roll = (_p_flow * error + _i_flow * _flow_x_integral);
    return std::clamp(roll, -kMaxPitchRoll, kMaxPitchRoll);
  }

  // ============== Sensor Data Callbacks ==============
  void customFlowCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) {
      RCLCPP_WARN_THROTTLE(node().get_logger(), *node().get_clock(), 5000, "Custom flow message has less than 3 elements.");
      return;
    }

    // Get latest offsets
    node().get_parameter("offsets.flow_x", _flow_offset_x);
    node().get_parameter("offsets.flow_y", _flow_offset_y);

    // Raw values from the new sensor
    const float raw_flow_u = msg->data[0]; // backwards is negative
    const float raw_flow_v = msg->data[1]; // left is positive
    const float raw_divergence = msg->data[2];

    // Apply offsets and coordinate transformations
    const float corrected_flow_x = -(raw_flow_v - _flow_offset_y);
    const float corrected_flow_y = (raw_flow_u - _flow_offset_x);

    // Apply low-pass filter to all measurements
    node().get_parameter("filters.sensor_alpha", _sensor_filter_alpha);
    _flow_x_rad_s = _sensor_filter_alpha * corrected_flow_x + (1.0 - _sensor_filter_alpha) * _flow_x_rad_s;
    _flow_y_rad_s = _sensor_filter_alpha * corrected_flow_y + (1.0 - _sensor_filter_alpha) * _flow_y_rad_s;
    _divergence = _sensor_filter_alpha * raw_divergence + (1.0 - _sensor_filter_alpha) * _divergence; //leaving this out. 
    //_divergence = raw_divergence;

    // --- NEW: very slow bias learner while craft is near-perfectly still
    if (fabs(_flow_x_rad_s) < 0.02f && fabs(_flow_y_rad_s) < 0.02f && fabs(_divergence) < 0.01f) {
      _flow_offset_x = 0.999 * _flow_offset_x + 0.001 * raw_flow_u;
      _flow_offset_y = 0.999 * _flow_offset_y + 0.001 * raw_flow_v;
    }

    // Print the filtered values for debugging
    RCLCPP_INFO_THROTTLE(
      node().get_logger(), *node().get_clock(), 1000, // Log every 1000ms (1s)
      "Filtered -> Flow X: %.3f, Flow Y: %.3f, Divergence: %.4f", _flow_x_rad_s, _flow_y_rad_s, _divergence);
  }

  void distanceSensorCallback(const px4_msgs::msg::DistanceSensor::SharedPtr msg)
  {
    // Only update the current distance for the safety check
    if (std::isfinite(msg->current_distance)) {
      _current_distance_m = msg->current_distance;
    }
  }
  void hoverThrustCallback(const px4_msgs::msg::HoverThrustEstimate::SharedPtr msg);


  // Setpoint type required for registration
  std::shared_ptr<px4_ros2::AttitudeSetpointType> _attitude_setpoint;

  // Subscribers
  rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr _distance_sensor_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr _custom_flow_sub;
  rclcpp::Subscription<px4_msgs::msg::HoverThrustEstimate>::SharedPtr _hover_thrust_sub; // for thrust from px4.

  // State variables
  float _yaw{0.f};

  // Filtered sensor values
  float _current_distance_m{-1.f};
  float _h_setpoint{1.f};          // default until AUTO engages
  float _flow_x_rad_s{0.f};
  float _flow_y_rad_s{0.f};
  float _divergence{0.f};

  // --- NEW: base gains for velocity control ---
  double _kp0_vel, _ki0_vel;

  // PI controller integrals
  float _vertical_integral{0.f};
  float _flow_x_integral{0.f};
  float _flow_y_integral{0.f};

};

 /* ---- hover-thrust callback ---- */
void FlightModeTest::hoverThrustCallback(const px4_msgs::msg::HoverThrustEstimate::SharedPtr msg)
{
  if (std::isfinite(msg->hover_thrust)) {     // sanity-check
       _hover_thrust = msg->hover_thrust;      // copy 0 … 1 value
  }
} 


