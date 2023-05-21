/**
Software License Agreement (BSD)
\file      create_driver.cpp
\authors   Jacob Perron <jacobmperron@gmail.com>
\copyright Copyright (c) 2015, Autonomy Lab (Simon Fraser University), All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
 * Neither the name of Autonomy Lab nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include "create_driver/create_driver.h"

#include <chrono>
#include <memory>
#include <string>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

CreateDriver::CreateDriver()
: Node("create_driver"),
  model_(create::RobotModel::CREATE_2),
  tf_broadcaster_(this),
  diagnostics_(this),
  last_cmd_vel_time_(0),
  is_running_slowly_(false),
  latch_duration_(std::chrono::nanoseconds{0})
{
  dev_ = declare_parameter<std::string>("dev", "/dev/ttyUSB0");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  latch_duration_ = rclcpp::Duration::from_seconds(declare_parameter<double>("latch_cmd_duration", 0.2));
  loop_hz_ = declare_parameter<double>("loop_hz", 10.0);
  publish_tf_ = declare_parameter<bool>("publish_tf", true);
  oi_mode_workaround_ = declare_parameter<bool>("oi_mode_workaround", false);

  auto robot_model_name = declare_parameter<std::string>("robot_model", "CREATE_2");
  if (robot_model_name == "ROOMBA_400") {
    model_ = create::RobotModel::ROOMBA_400;
  } else if (robot_model_name == "CREATE_1") {
    model_ = create::RobotModel::CREATE_1;
  } else if (robot_model_name == "CREATE_2") {
    model_ = create::RobotModel::CREATE_2;
  } else {
    RCLCPP_FATAL_STREAM(get_logger(), "[CREATE] Robot model \"" + robot_model_name + "\" is not known.");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO_STREAM(get_logger(), "[CREATE] \"" << robot_model_name << "\" selected");

  baud_ = declare_parameter<int>("baud", model_.getBaud());

  // Disable signal handler; let rclcpp handle them
  robot_ = new create::Create(model_, false);

  // Enable/disable the OI mode reporting workaround in libcreate.
  // https://github.com/AutonomyLab/create_robot/issues/64
  robot_->setModeReportWorkaround(oi_mode_workaround_);

  if (!robot_->connect(dev_, baud_)) {
    RCLCPP_FATAL(get_logger(), "[CREATE] Failed to establish serial connection with Create.");
    rclcpp::shutdown();
  }

  RCLCPP_INFO(this->get_logger(), "[CREATE] Connection established.");

  // Start in full control mode
  robot_->setMode(create::MODE_FULL);

  // Show robot's battery level
  RCLCPP_INFO(
    get_logger(),
    "[CREATE] Battery level %.2f %%",
    (robot_->getBatteryCharge() / robot_->getBatteryCapacity()) * 100.0);

  // Set frame_id's
  mode_msg_.header.frame_id = base_frame_;
  bumper_msg_.header.frame_id = base_frame_;
  cliff_msg_.header.frame_id = base_frame_;
  charging_state_msg_.header.frame_id = base_frame_;
  tf_odom_.header.frame_id = odom_frame_;
  tf_odom_.child_frame_id = base_frame_;
  odom_msg_.header.frame_id = odom_frame_;
  odom_msg_.child_frame_id = base_frame_;
  joint_state_msg_.name.resize(2);
  joint_state_msg_.position.resize(2);
  joint_state_msg_.velocity.resize(2);
  joint_state_msg_.effort.resize(2);
  joint_state_msg_.name[0] = "left_wheel_joint";
  joint_state_msg_.name[1] = "right_wheel_joint";

  // Populate intial covariances
  for (int i = 0; i < 36; i++) {
    odom_msg_.pose.covariance[i] = COVARIANCE[i];
    odom_msg_.twist.covariance[i] = COVARIANCE[i];
  }

  // Setup subscribers
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 1, std::bind(&CreateDriver::cmdVelCallback, this, std::placeholders::_1));
  debris_led_sub_ = create_subscription<std_msgs::msg::Bool>(
    "debris_led", 10, std::bind(&CreateDriver::debrisLEDCallback, this, std::placeholders::_1));
  spot_led_sub_ = create_subscription<std_msgs::msg::Bool>(
    "spot_led", 10, std::bind(&CreateDriver::spotLEDCallback, this, std::placeholders::_1));
  dock_led_sub_ = create_subscription<std_msgs::msg::Bool>(
    "dock_led", 10, std::bind(&CreateDriver::dockLEDCallback, this, std::placeholders::_1));
  check_led_sub_ = create_subscription<std_msgs::msg::Bool>(
    "check_led", 10, std::bind(&CreateDriver::checkLEDCallback, this, std::placeholders::_1));
  power_led_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
    "power_led", 10, std::bind(&CreateDriver::powerLEDCallback, this, std::placeholders::_1));
  set_ascii_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
    "set_ascii", 10, std::bind(&CreateDriver::setASCIICallback, this, std::placeholders::_1));
  dock_sub_ = create_subscription<std_msgs::msg::Empty>(
    "dock", 10, std::bind(&CreateDriver::dockCallback, this, std::placeholders::_1));
  undock_sub_ = create_subscription<std_msgs::msg::Empty>(
    "undock", 10, std::bind(&CreateDriver::undockCallback, this, std::placeholders::_1));
  define_song_sub_ = create_subscription<create_msgs::msg::DefineSong>(
    "define_song", 10, std::bind(&CreateDriver::defineSongCallback, this, std::placeholders::_1));
  play_song_sub_ = create_subscription<create_msgs::msg::PlaySong>(
    "play_song", 10, std::bind(&CreateDriver::playSongCallback, this, std::placeholders::_1));
  side_brush_motor_sub_ = create_subscription<create_msgs::msg::MotorSetpoint>(
    "side_brush_motor", 10, std::bind(&CreateDriver::sideBrushMotor, this, std::placeholders::_1));
  main_brush_motor_sub_ = create_subscription<create_msgs::msg::MotorSetpoint>(
    "main_brush_motor", 10, std::bind(&CreateDriver::mainBrushMotor, this, std::placeholders::_1));
  vacuum_motor_sub_ = create_subscription<create_msgs::msg::MotorSetpoint>(
    "vacuum_motor", 10, std::bind(&CreateDriver::vacuumBrushMotor, this, std::placeholders::_1));

  // Setup publishers
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 30);
  clean_btn_pub_ = create_publisher<std_msgs::msg::Empty>("clean_button", 30);
  day_btn_pub_ = create_publisher<std_msgs::msg::Empty>("day_button", 30);
  hour_btn_pub_ = create_publisher<std_msgs::msg::Empty>("hour_button", 30);
  min_btn_pub_ = create_publisher<std_msgs::msg::Empty>("minute_button", 30);
  dock_btn_pub_ = create_publisher<std_msgs::msg::Empty>("dock_button", 30);
  spot_btn_pub_ = create_publisher<std_msgs::msg::Empty>("spot_button", 30);
  voltage_pub_ = create_publisher<std_msgs::msg::Float32>("battery/voltage", 30);
  current_pub_ = create_publisher<std_msgs::msg::Float32>("battery/current", 30);
  charge_pub_ = create_publisher<std_msgs::msg::Float32>("battery/charge", 30);
  charge_ratio_pub_ = create_publisher<std_msgs::msg::Float32>("battery/charge_ratio", 30);
  capacity_pub_ = create_publisher<std_msgs::msg::Float32>("battery/capacity", 30);
  temperature_pub_ = create_publisher<std_msgs::msg::Int16>("battery/temperature", 30);
  charging_state_pub_ = create_publisher<create_msgs::msg::ChargingState>("battery/charging_state", 30);
  omni_char_pub_ = create_publisher<std_msgs::msg::UInt16>("ir_omni", 30);
  mode_pub_ = create_publisher<create_msgs::msg::Mode>("mode", 30);
  bumper_pub_ = create_publisher<create_msgs::msg::Bumper>("bumper", 30);
  cliff_pub_ = create_publisher<create_msgs::msg::Cliff>("cliff", 30);
  wheeldrop_pub_ = create_publisher<std_msgs::msg::Empty>("wheeldrop", 30);
  wheel_joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  // Setup diagnostics
  diagnostics_.add("Battery Status", this, &CreateDriver::updateBatteryDiagnostics);
  diagnostics_.add("Safety Status", this, &CreateDriver::updateSafetyDiagnostics);
  diagnostics_.add("Serial Status", this, &CreateDriver::updateSerialDiagnostics);
  diagnostics_.add("Base Mode", this, &CreateDriver::updateModeDiagnostics);
  diagnostics_.add("Driver Status", this, &CreateDriver::updateDriverDiagnostics);

  diagnostics_.setHardwareID(robot_model_name);

  // Setup update loop.
  const auto loop_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / loop_hz_));
  loop_timer_ = create_wall_timer(loop_period, std::bind(&CreateDriver::spinOnce, this));

  RCLCPP_INFO(get_logger(), "[CREATE] Ready.");
}

CreateDriver::~CreateDriver()
{
  RCLCPP_INFO(get_logger(), "[CREATE] Destruct sequence initiated.");
  robot_->disconnect();
  delete robot_;
}

void CreateDriver::cmdVelCallback(geometry_msgs::msg::Twist::UniquePtr msg)
{
  robot_->drive(msg->linear.x, msg->angular.z);
  last_cmd_vel_time_ = now();
}

void CreateDriver::debrisLEDCallback(std_msgs::msg::Bool::UniquePtr msg)
{
  robot_->enableDebrisLED(msg->data);
}

void CreateDriver::spotLEDCallback(std_msgs::msg::Bool::UniquePtr msg)
{
  robot_->enableSpotLED(msg->data);
}

void CreateDriver::dockLEDCallback(std_msgs::msg::Bool::UniquePtr msg)
{
  robot_->enableDockLED(msg->data);
}

void CreateDriver::checkLEDCallback(std_msgs::msg::Bool::UniquePtr msg)
{
  robot_->enableCheckRobotLED(msg->data);
}

void CreateDriver::powerLEDCallback(std_msgs::msg::UInt8MultiArray::UniquePtr msg)
{
  if (msg->data.size() < 1) {
    RCLCPP_ERROR(get_logger(), "[CREATE] No values provided to set power LED");
    return;
  }
  if (msg->data.size() < 2) {
    robot_->setPowerLED(msg->data[0]);
  } else {
    robot_->setPowerLED(msg->data[0], msg->data[1]);
  }
}

void CreateDriver::setASCIICallback(std_msgs::msg::UInt8MultiArray::UniquePtr msg)
{
  bool result;
  if (msg->data.size() < 1) {
    RCLCPP_ERROR(get_logger(), "[CREATE] No ASCII digits provided");
    return;
  }
  if (msg->data.size() < 2) {
    result = robot_->setDigitsASCII(msg->data[0], ' ', ' ', ' ');
  } else if (msg->data.size() < 3) {
    result = robot_->setDigitsASCII(msg->data[0], msg->data[1], ' ', ' ');
  } else if (msg->data.size() < 4) {
    result = robot_->setDigitsASCII(msg->data[0], msg->data[1], msg->data[2], ' ');
  } else {
    result = robot_->setDigitsASCII(msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
  }

  if (!result) {
    RCLCPP_ERROR(get_logger(), "[CREATE] ASCII character out of range [32, 126]");
  }
}

void CreateDriver::dockCallback(std_msgs::msg::Empty::UniquePtr msg)
{
  (void) msg;

  robot_->setMode(create::MODE_PASSIVE);

  if (model_.getVersion() <= create::V_2) {
    usleep(1000000);  // Create 1 requires a delay (1 sec)
  }

  // Call docking behaviour
  robot_->dock();
}

void CreateDriver::undockCallback(std_msgs::msg::Empty::UniquePtr msg)
{
  (void) msg;

  // Switch robot back to FULL mode
  robot_->setMode(create::MODE_FULL);
}

void CreateDriver::defineSongCallback(create_msgs::msg::DefineSong::UniquePtr msg)
{
  if (!robot_->defineSong(msg->song, msg->length, &(msg->notes.front()), &(msg->durations.front()))) {
    RCLCPP_ERROR_STREAM(get_logger(), "[CREATE] Failed to define song " << msg->song << " of length " << msg->length);
  }
}

void CreateDriver::playSongCallback(create_msgs::msg::PlaySong::UniquePtr msg)
{
  if (!robot_->playSong(msg->song)) {
    RCLCPP_ERROR_STREAM(get_logger(), "[CREATE] Failed to play song " << msg->song);
  }
}

void CreateDriver::sideBrushMotor(create_msgs::msg::MotorSetpoint::UniquePtr msg)
{
  if (!robot_->setSideMotor(msg->duty_cycle)) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "[CREATE] Failed to set duty cycle " << msg->duty_cycle << " for side brush motor");
  }
}

void CreateDriver::mainBrushMotor(create_msgs::msg::MotorSetpoint::UniquePtr msg)
{
  if (!robot_->setMainMotor(msg->duty_cycle)) {
    RCLCPP_ERROR_STREAM(get_logger(), "[CREATE] Failed to set duty cycle " << msg->duty_cycle << " for main motor");
  }
}

void CreateDriver::vacuumBrushMotor(create_msgs::msg::MotorSetpoint::UniquePtr msg)
{
  if (!robot_->setVacuumMotor(msg->duty_cycle)) {
    RCLCPP_ERROR_STREAM(get_logger(), "[CREATE] Failed to set duty cycle " << msg->duty_cycle << " for vacuum motor");
  }
}

bool CreateDriver::update()
{
  publishOdom();
  publishJointState();
  publishBatteryInfo();
  publishButtonPresses();
  publishOmniChar();
  publishMode();
  publishBumperInfo();
  publishWheeldrop();
  publishCliff();

  // If last velocity command was sent longer than latch duration, stop robot
  if (last_cmd_vel_time_.nanoseconds() == 0 || now() - last_cmd_vel_time_ >= latch_duration_) {
    robot_->drive(0, 0);
  }

  return true;
}

void CreateDriver::updateBatteryDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  const float charge = robot_->getBatteryCharge();
  const float capacity = robot_->getBatteryCapacity();
  const create::ChargingState charging_state = robot_->getChargingState();
  const float charge_ratio = charge / capacity;

  if (charging_state == create::CHARGE_FAULT) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Charging fault reported by base");
  } else if (charge_ratio == 0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Battery reports no charge");
  } else if (charge_ratio < 0.1) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Battery reports less than 10% charge");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Battery is OK");
  }

  stat.add("Charge (Ah)", charge);
  stat.add("Capacity (Ah)", capacity);
  stat.add("Temperature (Celsius)", robot_->getTemperature());
  stat.add("Current (A)", robot_->getCurrent());
  stat.add("Voltage (V)", robot_->getVoltage());

  switch (charging_state) {
    case create::CHARGE_NONE:
      stat.add("Charging state", "Not charging");
      break;
    case create::CHARGE_RECONDITION:
      stat.add("Charging state", "Reconditioning");
      break;
    case create::CHARGE_FULL:
      stat.add("Charging state", "Full charge");
      break;
    case create::CHARGE_TRICKLE:
      stat.add("Charging state", "Trickle charging");
      break;
    case create::CHARGE_WAITING:
      stat.add("Charging state", "Waiting");
      break;
    case create::CHARGE_FAULT:
      stat.add("Charging state", "Fault");
      break;
  }
}

void CreateDriver::updateSafetyDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  const bool is_wheeldrop = robot_->isWheeldrop();
  const bool is_cliff = robot_->isCliff();
  if (is_wheeldrop) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Wheeldrop detected");
  } else if (is_cliff) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Cliff detected");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "No safety issues detected");
  }

  stat.add("Wheeldrop", is_wheeldrop);
  stat.add("Cliff", is_cliff);
}

void CreateDriver::updateSerialDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  const bool is_connected = robot_->connected();
  const uint64_t corrupt_packets = robot_->getNumCorruptPackets();
  const uint64_t total_packets = robot_->getTotalPackets();

  if (!is_connected) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Serial port to base not open");
  } else if (corrupt_packets) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                 "Corrupt packets detected. If the number of corrupt packets is increasing, data may be unreliable");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Serial connection is good");
  }

  stat.add("Corrupt packets", corrupt_packets);
  stat.add("Total packets", total_packets);
}

void CreateDriver::updateModeDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  const create::CreateMode mode = robot_->getMode();
  switch (mode) {
    case create::MODE_UNAVAILABLE:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Unknown mode of operation");
      break;
    case create::MODE_OFF:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Mode is set to OFF");
      break;
    case create::MODE_PASSIVE:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Mode is set to PASSIVE");
      break;
    case create::MODE_SAFE:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Mode is set to SAFE");
      break;
    case create::MODE_FULL:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Mode is set to FULL");
      break;
  }
}

void CreateDriver::updateDriverDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (is_running_slowly_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Internal loop running slowly");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Maintaining loop frequency");
  }
}

void CreateDriver::publishOdom()
{
  create::Pose pose = robot_->getPose();
  create::Vel vel = robot_->getVel();

  // Populate position info
  tf2::Quaternion tf_quat;
  tf_quat.setRPY(0.0, 0.0, pose.yaw);
  geometry_msgs::msg::Quaternion quat = tf2::toMsg(tf_quat);
  odom_msg_.header.stamp = now();
  odom_msg_.pose.pose.position.x = pose.x;
  odom_msg_.pose.pose.position.y = pose.y;
  odom_msg_.pose.pose.orientation = quat;

  // Populate velocity info
  odom_msg_.twist.twist.linear.x = vel.x;
  odom_msg_.twist.twist.linear.y = vel.y;
  odom_msg_.twist.twist.angular.z = vel.yaw;

  // Update covariances
  odom_msg_.pose.covariance[0] = static_cast<double>(pose.covariance[0]);
  odom_msg_.pose.covariance[1] = pose.covariance[1];
  odom_msg_.pose.covariance[5] = pose.covariance[2];
  odom_msg_.pose.covariance[6] = pose.covariance[3];
  odom_msg_.pose.covariance[7] = pose.covariance[4];
  odom_msg_.pose.covariance[11] = pose.covariance[5];
  odom_msg_.pose.covariance[30] = pose.covariance[6];
  odom_msg_.pose.covariance[31] = pose.covariance[7];
  odom_msg_.pose.covariance[35] = pose.covariance[8];
  odom_msg_.twist.covariance[0] = vel.covariance[0];
  odom_msg_.twist.covariance[1] = vel.covariance[1];
  odom_msg_.twist.covariance[5] = vel.covariance[2];
  odom_msg_.twist.covariance[6] = vel.covariance[3];
  odom_msg_.twist.covariance[7] = vel.covariance[4];
  odom_msg_.twist.covariance[11] = vel.covariance[5];
  odom_msg_.twist.covariance[30] = vel.covariance[6];
  odom_msg_.twist.covariance[31] = vel.covariance[7];
  odom_msg_.twist.covariance[35] = vel.covariance[8];

  if (publish_tf_) {
    tf_odom_.header.stamp = now();
    tf_odom_.transform.translation.x = pose.x;
    tf_odom_.transform.translation.y = pose.y;
    tf_odom_.transform.rotation = quat;
    tf_broadcaster_.sendTransform(tf_odom_);
  }

  odom_pub_->publish(odom_msg_);
}

void CreateDriver::publishJointState()
{
  // Publish joint states
  float wheelRadius = model_.getWheelDiameter() / 2.0;

  joint_state_msg_.header.stamp = now();
  joint_state_msg_.position[0] = robot_->getLeftWheelDistance() / wheelRadius;
  joint_state_msg_.position[1] = robot_->getRightWheelDistance() / wheelRadius;
  joint_state_msg_.velocity[0] = robot_->getRequestedLeftWheelVel() / wheelRadius;
  joint_state_msg_.velocity[1] = robot_->getRequestedRightWheelVel() / wheelRadius;
  wheel_joint_pub_->publish(joint_state_msg_);
}

void CreateDriver::publishBatteryInfo()
{
  float32_msg_.data = robot_->getVoltage();
  voltage_pub_->publish(float32_msg_);
  float32_msg_.data = robot_->getCurrent();
  current_pub_->publish(float32_msg_);
  float32_msg_.data = robot_->getBatteryCharge();
  charge_pub_->publish(float32_msg_);
  float32_msg_.data = robot_->getBatteryCapacity();
  capacity_pub_->publish(float32_msg_);
  int16_msg_.data = robot_->getTemperature();
  temperature_pub_->publish(int16_msg_);
  float32_msg_.data = robot_->getBatteryCharge() / robot_->getBatteryCapacity();
  charge_ratio_pub_->publish(float32_msg_);

  const create::ChargingState charging_state = robot_->getChargingState();
  charging_state_msg_.header.stamp = now();
  switch (charging_state) {
    case create::CHARGE_NONE:
      charging_state_msg_.state = charging_state_msg_.CHARGE_NONE;
      break;
    case create::CHARGE_RECONDITION:
      charging_state_msg_.state = charging_state_msg_.CHARGE_RECONDITION;
      break;

    case create::CHARGE_FULL:
      charging_state_msg_.state = charging_state_msg_.CHARGE_FULL;
      break;

    case create::CHARGE_TRICKLE:
      charging_state_msg_.state = charging_state_msg_.CHARGE_TRICKLE;
      break;

    case create::CHARGE_WAITING:
      charging_state_msg_.state = charging_state_msg_.CHARGE_WAITING;
      break;

    case create::CHARGE_FAULT:
      charging_state_msg_.state = charging_state_msg_.CHARGE_FAULT;
      break;
  }
  charging_state_pub_->publish(charging_state_msg_);
}

void CreateDriver::publishButtonPresses() const
{
  if (robot_->isCleanButtonPressed()) {
    clean_btn_pub_->publish(empty_msg_);
  }
  if (robot_->isDayButtonPressed()) {
    day_btn_pub_->publish(empty_msg_);
  }
  if (robot_->isHourButtonPressed()) {
    hour_btn_pub_->publish(empty_msg_);
  }
  if (robot_->isMinButtonPressed()) {
    min_btn_pub_->publish(empty_msg_);
  }
  if (robot_->isDockButtonPressed()) {
    dock_btn_pub_->publish(empty_msg_);
  }
  if (robot_->isSpotButtonPressed()) {
    spot_btn_pub_->publish(empty_msg_);
  }
}

void CreateDriver::publishOmniChar()
{
  uint8_t ir_char = robot_->getIROmni();
  uint16_msg_.data = ir_char;
  omni_char_pub_->publish(uint16_msg_);
  // TODO(jacobperron): Publish info based on character, such as dock in sight
}

void CreateDriver::publishMode()
{
  const create::CreateMode mode = robot_->getMode();
  mode_msg_.header.stamp = now();
  switch (mode) {
    case create::MODE_OFF:
      mode_msg_.mode = mode_msg_.MODE_OFF;
      break;
    case create::MODE_PASSIVE:
      mode_msg_.mode = mode_msg_.MODE_PASSIVE;
      break;
    case create::MODE_SAFE:
      mode_msg_.mode = mode_msg_.MODE_SAFE;
      break;
    case create::MODE_FULL:
      mode_msg_.mode = mode_msg_.MODE_FULL;
      break;
    default:
      RCLCPP_ERROR(get_logger(), "[CREATE] Unknown mode detected");
      break;
  }
  mode_pub_->publish(mode_msg_);
}

void CreateDriver::publishBumperInfo()
{
  bumper_msg_.header.stamp = now();
  bumper_msg_.is_left_pressed = robot_->isLeftBumper();
  bumper_msg_.is_right_pressed = robot_->isRightBumper();

  if (model_.getVersion() >= create::V_3) {
    bumper_msg_.is_light_left = robot_->isLightBumperLeft();
    bumper_msg_.is_light_front_left = robot_->isLightBumperFrontLeft();
    bumper_msg_.is_light_center_left = robot_->isLightBumperCenterLeft();
    bumper_msg_.is_light_right = robot_->isLightBumperRight();
    bumper_msg_.is_light_front_right = robot_->isLightBumperFrontRight();
    bumper_msg_.is_light_center_right = robot_->isLightBumperCenterRight();

    bumper_msg_.light_signal_left = robot_->getLightSignalLeft();
    bumper_msg_.light_signal_front_left = robot_->getLightSignalFrontLeft();
    bumper_msg_.light_signal_center_left = robot_->getLightSignalCenterLeft();
    bumper_msg_.light_signal_right = robot_->getLightSignalRight();
    bumper_msg_.light_signal_front_right = robot_->getLightSignalFrontRight();
    bumper_msg_.light_signal_center_right = robot_->getLightSignalCenterRight();
  }

  bumper_pub_->publish(bumper_msg_);
}

void CreateDriver::publishWheeldrop()
{
  if (robot_->isWheeldrop()) {
    wheeldrop_pub_->publish(empty_msg_);
  }
}

void CreateDriver::publishCliff()
{
  cliff_msg_.header.stamp = now();
  cliff_msg_.is_cliff_left = robot_->isCliffLeft();
  cliff_msg_.is_cliff_front_left = robot_->isCliffFrontLeft();
  cliff_msg_.is_cliff_right = robot_->isCliffRight();
  cliff_msg_.is_cliff_front_right = robot_->isCliffFrontRight();
  cliff_pub_->publish(cliff_msg_);
}

void CreateDriver::spinOnce()
{
  const auto spin_start = now();

  update();
  diagnostics_.force_update();

  // Check if the spin took longer than the target loop period.
  const auto spin_end = now();
  const auto elapsed = spin_end - spin_start;
  const double target_period = 1. / loop_hz_;
  is_running_slowly_ = elapsed.seconds() > target_period;

  if (is_running_slowly_) {
    RCLCPP_WARN(get_logger(), "[CREATE] Loop running slowly.");
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto create_driver = std::make_shared<CreateDriver>();

  try {
    rclcpp::spin(create_driver);
  } catch (std::runtime_error& ex) {
    RCLCPP_FATAL_STREAM(create_driver->get_logger(), "[CREATE] Runtime error: " << ex.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
