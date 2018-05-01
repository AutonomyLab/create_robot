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

#include <tf/transform_datatypes.h>

#include <string>

CreateDriver::CreateDriver(ros::NodeHandle& nh)
  : nh_(nh),
    priv_nh_("~"),
    diagnostics_(),
    model_(create::RobotModel::CREATE_2),
    is_running_slowly_(false)
{
  bool create_one;
  std::string robot_model_name;
  priv_nh_.param<std::string>("dev", dev_, "/dev/ttyUSB0");
  priv_nh_.param<std::string>("robot_model", robot_model_name, "CREATE_2");
  priv_nh_.param<std::string>("base_frame", base_frame_, "base_footprint");
  priv_nh_.param<std::string>("odom_frame", odom_frame_, "odom");
  priv_nh_.param<double>("latch_cmd_duration", latch_duration_, 0.2);
  priv_nh_.param<double>("loop_hz", loop_hz_, 10.0);
  priv_nh_.param<bool>("publish_tf", publish_tf_, true);

  if (robot_model_name == "ROOMBA_400")
  {
    model_ = create::RobotModel::ROOMBA_400;
  }
  else if (robot_model_name == "CREATE_1")
  {
    model_ = create::RobotModel::CREATE_1;
  }
  else if (robot_model_name == "CREATE_2")
  {
    model_ = create::RobotModel::CREATE_2;
  }
  else
  {
    ROS_FATAL_STREAM("[CREATE] Robot model \"" + robot_model_name + "\" is not known.");
    ros::shutdown();
    return;
  }

  ROS_INFO_STREAM("[CREATE] \"" << robot_model_name << "\" selected");

  priv_nh_.param<int>("baud", baud_, model_.getBaud());

  robot_ = new create::Create(model_);

  if (!robot_->connect(dev_, baud_))
  {
    ROS_FATAL("[CREATE] Failed to establish serial connection with Create.");
    ros::shutdown();
  }

  ROS_INFO("[CREATE] Connection established.");

  // Start in full control mode
  robot_->setMode(create::MODE_FULL);

  // Show robot's battery level
  ROS_INFO("[CREATE] Battery level %.2f %%", (robot_->getBatteryCharge() / robot_->getBatteryCapacity()) * 100.0);

  // Set frame_id's
  mode_msg_.header.frame_id = base_frame_;
  bumper_msg_.header.frame_id = base_frame_;
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
  for (int i = 0; i < 36; i++)
  {
    odom_msg_.pose.covariance[i] = COVARIANCE[i];
    odom_msg_.twist.covariance[i] = COVARIANCE[i];
  }

  // Setup subscribers
  cmd_vel_sub_ = nh.subscribe("cmd_vel", 1, &CreateDriver::cmdVelCallback, this);
  debris_led_sub_ = nh.subscribe("debris_led", 10, &CreateDriver::debrisLEDCallback, this);
  spot_led_sub_ = nh.subscribe("spot_led", 10, &CreateDriver::spotLEDCallback, this);
  dock_led_sub_ = nh.subscribe("dock_led", 10, &CreateDriver::dockLEDCallback, this);
  check_led_sub_ = nh.subscribe("check_led", 10, &CreateDriver::checkLEDCallback, this);
  power_led_sub_ = nh.subscribe("power_led", 10, &CreateDriver::powerLEDCallback, this);
  set_ascii_sub_ = nh.subscribe("set_ascii", 10, &CreateDriver::setASCIICallback, this);
  dock_sub_ = nh.subscribe("dock", 10, &CreateDriver::dockCallback, this);
  undock_sub_ = nh.subscribe("undock", 10, &CreateDriver::undockCallback, this);
  define_song_sub_ = nh.subscribe("define_song", 10, &CreateDriver::defineSongCallback, this);
  play_song_sub_ = nh.subscribe("play_song", 10, &CreateDriver::playSongCallback, this);

  // Setup publishers
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 30);
  clean_btn_pub_ = nh.advertise<std_msgs::Empty>("clean_button", 30);
  day_btn_pub_ = nh.advertise<std_msgs::Empty>("day_button", 30);
  hour_btn_pub_ = nh.advertise<std_msgs::Empty>("hour_button", 30);
  min_btn_pub_ = nh.advertise<std_msgs::Empty>("minute_button", 30);
  dock_btn_pub_ = nh.advertise<std_msgs::Empty>("dock_button", 30);
  spot_btn_pub_ = nh.advertise<std_msgs::Empty>("spot_button", 30);
  voltage_pub_ = nh.advertise<std_msgs::Float32>("battery/voltage", 30);
  current_pub_ = nh.advertise<std_msgs::Float32>("battery/current", 30);
  charge_pub_ = nh.advertise<std_msgs::Float32>("battery/charge", 30);
  charge_ratio_pub_ = nh.advertise<std_msgs::Float32>("battery/charge_ratio", 30);
  capacity_pub_ = nh.advertise<std_msgs::Float32>("battery/capacity", 30);
  temperature_pub_ = nh.advertise<std_msgs::Int16>("battery/temperature", 30);
  charging_state_pub_ = nh.advertise<ca_msgs::ChargingState>("battery/charging_state", 30);
  omni_char_pub_ = nh.advertise<std_msgs::UInt16>("ir_omni", 30);
  mode_pub_ = nh.advertise<ca_msgs::Mode>("mode", 30);
  bumper_pub_ = nh.advertise<ca_msgs::Bumper>("bumper", 30);
  wheeldrop_pub_ = nh.advertise<std_msgs::Empty>("wheeldrop", 30);
  wheel_joint_pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

  // Setup diagnostics
  diagnostics_.add("Battery Status", this, &CreateDriver::updateBatteryDiagnostics);
  diagnostics_.add("Safety Status", this, &CreateDriver::updateSafetyDiagnostics);
  diagnostics_.add("Serial Status", this, &CreateDriver::updateSerialDiagnostics);
  diagnostics_.add("Base Mode", this, &CreateDriver::updateModeDiagnostics);
  diagnostics_.add("Driver Status", this, &CreateDriver::updateDriverDiagnostics);

  diagnostics_.setHardwareID(robot_model_name);

  ROS_INFO("[CREATE] Ready.");
}

CreateDriver::~CreateDriver()
{
  ROS_INFO("[CREATE] Destruct sequence initiated.");
  robot_->disconnect();
  delete robot_;
}

void CreateDriver::cmdVelCallback(const geometry_msgs::TwistConstPtr& msg)
{
  robot_->drive(msg->linear.x, msg->angular.z);
  last_cmd_vel_time_ = ros::Time::now();
}

void CreateDriver::debrisLEDCallback(const std_msgs::BoolConstPtr& msg)
{
  robot_->enableDebrisLED(msg->data);
}

void CreateDriver::spotLEDCallback(const std_msgs::BoolConstPtr& msg)
{
  robot_->enableSpotLED(msg->data);
}

void CreateDriver::dockLEDCallback(const std_msgs::BoolConstPtr& msg)
{
  robot_->enableDockLED(msg->data);
}

void CreateDriver::checkLEDCallback(const std_msgs::BoolConstPtr& msg)
{
  robot_->enableCheckRobotLED(msg->data);
}

void CreateDriver::powerLEDCallback(const std_msgs::UInt8MultiArrayConstPtr& msg)
{
  if (msg->data.size() < 1)
  {
    ROS_ERROR("[CREATE] No values provided to set power LED");
  }
  else
  {
    if (msg->data.size() < 2)
    {
      robot_->setPowerLED(msg->data[0]);
    }
    else
    {
      robot_->setPowerLED(msg->data[0], msg->data[1]);
    }
  }
}

void CreateDriver::setASCIICallback(const std_msgs::UInt8MultiArrayConstPtr& msg)
{
  bool result;
  if (msg->data.size() < 1)
  {
    ROS_ERROR("[CREATE] No ASCII digits provided");
  }
  else if (msg->data.size() < 2)
  {
    result = robot_->setDigitsASCII(msg->data[0], ' ', ' ', ' ');
  }
  else if (msg->data.size() < 3)
  {
    result = robot_->setDigitsASCII(msg->data[0], msg->data[1], ' ', ' ');
  }
  else if (msg->data.size() < 4)
  {
    result = robot_->setDigitsASCII(msg->data[0], msg->data[1], msg->data[2], ' ');
  }
  else
  {
    result = robot_->setDigitsASCII(msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
  }

  if (!result)
  {
    ROS_ERROR("[CREATE] ASCII character out of range [32, 126]");
  }
}

void CreateDriver::dockCallback(const std_msgs::EmptyConstPtr& msg)
{
  robot_->setMode(create::MODE_PASSIVE);

  if (model_.getVersion() <= create::V_2)
    usleep(1000000);  // Create 1 requires a delay (1 sec)

  // Call docking behaviour
  robot_->dock();
}

void CreateDriver::undockCallback(const std_msgs::EmptyConstPtr& msg)
{
  // Switch robot back to FULL mode
  robot_->setMode(create::MODE_FULL);
}

void CreateDriver::defineSongCallback(const ca_msgs::DefineSongConstPtr& msg)
{
  if (!robot_->defineSong(msg->song, msg->length, &(msg->notes.front()), &(msg->durations.front())))
  {
    ROS_ERROR_STREAM("[CREATE] Failed to define song " << msg->song << " of length " << msg->length);
  }
}

void CreateDriver::playSongCallback(const ca_msgs::PlaySongConstPtr& msg)
{
  if (!robot_->playSong(msg->song))
  {
    ROS_ERROR_STREAM("[CREATE] Failed to play song " << msg->song);
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

  // If last velocity command was sent longer than latch duration, stop robot
  if (ros::Time::now() - last_cmd_vel_time_ >= ros::Duration(latch_duration_))
  {
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

  if (charging_state == create::CHARGE_FAULT)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Charging fault reported by base");
  }
  else if (charge_ratio == 0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery reports no charge");
  }
  else if (charge_ratio < 0.1)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Battery reports less than 10% charge");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Battery is OK");
  }

  stat.add("Charge (Ah)", charge);
  stat.add("Capacity (Ah)", capacity);
  stat.add("Temperature (Celsius)", robot_->getTemperature());
  stat.add("Current (A)", robot_->getCurrent());
  stat.add("Voltage (V)", robot_->getVoltage());

  switch (charging_state)
  {
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
  if (is_wheeldrop)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Wheeldrop detected");
  }
  else if (is_cliff)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Cliff detected");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "No safety issues detected");
  }

  stat.add("Wheeldrop", is_wheeldrop);
  stat.add("Cliff", is_cliff);
}

void CreateDriver::updateSerialDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  const bool is_connected = robot_->connected();
  const uint64_t corrupt_packets = robot_->getNumCorruptPackets();
  const uint64_t total_packets = robot_->getTotalPackets();

  if (!is_connected)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Serial port to base not open");
  }
  else if (corrupt_packets)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                 "Corrupt packets detected. If the number of corrupt packets is increasing, data may be unreliable");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Serial connection is good");
  }

  stat.add("Corrupt packets", corrupt_packets);
  stat.add("Total packets", total_packets);
}

void CreateDriver::updateModeDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  const create::CreateMode mode = robot_->getMode();
  switch (mode)
  {
    case create::MODE_UNAVAILABLE:
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Unknown mode of operation");
      break;
    case create::MODE_OFF:
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Mode is set to OFF");
      break;
    case create::MODE_PASSIVE:
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Mode is set to PASSIVE");
      break;
    case create::MODE_SAFE:
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Mode is set to SAFE");
      break;
    case create::MODE_FULL:
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Mode is set to FULL");
      break;
  }
}

void CreateDriver::updateDriverDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (is_running_slowly_)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Internal loop running slowly");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Maintaining loop frequency");
  }
}

void CreateDriver::publishOdom()
{
  create::Pose pose = robot_->getPose();
  create::Vel vel = robot_->getVel();

  // Populate position info
  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose.yaw);
  odom_msg_.header.stamp = ros::Time::now();
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

  if (publish_tf_)
  {
    tf_odom_.header.stamp = ros::Time::now();
    tf_odom_.transform.translation.x = pose.x;
    tf_odom_.transform.translation.y = pose.y;
    tf_odom_.transform.rotation = quat;
    tf_broadcaster_.sendTransform(tf_odom_);
  }

  odom_pub_.publish(odom_msg_);
}

void CreateDriver::publishJointState()
{
  // Publish joint states
  float wheelRadius = model_.getWheelDiameter() / 2.0;

  joint_state_msg_.header.stamp = ros::Time::now();
  joint_state_msg_.position[0] = robot_->getLeftWheelDistance() / wheelRadius;
  joint_state_msg_.position[1] = robot_->getRightWheelDistance() / wheelRadius;
  joint_state_msg_.velocity[0] = robot_->getRequestedLeftWheelVel() / wheelRadius;
  joint_state_msg_.velocity[1] = robot_->getRequestedRightWheelVel() / wheelRadius;
  wheel_joint_pub_.publish(joint_state_msg_);
}

void CreateDriver::publishBatteryInfo()
{
  float32_msg_.data = robot_->getVoltage();
  voltage_pub_.publish(float32_msg_);
  float32_msg_.data = robot_->getCurrent();
  current_pub_.publish(float32_msg_);
  float32_msg_.data = robot_->getBatteryCharge();
  charge_pub_.publish(float32_msg_);
  float32_msg_.data = robot_->getBatteryCapacity();
  capacity_pub_.publish(float32_msg_);
  int16_msg_.data = robot_->getTemperature();
  temperature_pub_.publish(int16_msg_);
  float32_msg_.data = robot_->getBatteryCharge() / robot_->getBatteryCapacity();
  charge_ratio_pub_.publish(float32_msg_);

  const create::ChargingState charging_state = robot_->getChargingState();
  charging_state_msg_.header.stamp = ros::Time::now();
  switch (charging_state)
  {
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
  charging_state_pub_.publish(charging_state_msg_);
}

void CreateDriver::publishButtonPresses() const
{
  if (robot_->isCleanButtonPressed())
  {
    clean_btn_pub_.publish(empty_msg_);
  }
  if (robot_->isDayButtonPressed())
  {
    day_btn_pub_.publish(empty_msg_);
  }
  if (robot_->isHourButtonPressed())
  {
    hour_btn_pub_.publish(empty_msg_);
  }
  if (robot_->isMinButtonPressed())
  {
    min_btn_pub_.publish(empty_msg_);
  }
  if (robot_->isDockButtonPressed())
  {
    dock_btn_pub_.publish(empty_msg_);
  }
  if (robot_->isSpotButtonPressed())
  {
    spot_btn_pub_.publish(empty_msg_);
  }
}

void CreateDriver::publishOmniChar()
{
  uint8_t ir_char = robot_->getIROmni();
  uint16_msg_.data = ir_char;
  omni_char_pub_.publish(uint16_msg_);
  // TODO(jacobperron): Publish info based on character, such as dock in sight
}

void CreateDriver::publishMode()
{
  const create::CreateMode mode = robot_->getMode();
  mode_msg_.header.stamp = ros::Time::now();
  switch (mode)
  {
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
      ROS_ERROR("[CREATE] Unknown mode detected");
      break;
  }
  mode_pub_.publish(mode_msg_);
}

void CreateDriver::publishBumperInfo()
{
  bumper_msg_.header.stamp = ros::Time::now();
  bumper_msg_.is_left_pressed = robot_->isLeftBumper();
  bumper_msg_.is_right_pressed = robot_->isRightBumper();

  if (model_.getVersion() >= create::V_3)
  {
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

  bumper_pub_.publish(bumper_msg_);
}

void CreateDriver::publishWheeldrop()
{
  if (robot_->isWheeldrop())
    wheeldrop_pub_.publish(empty_msg_);
}

void CreateDriver::spinOnce()
{
  update();
  diagnostics_.update();
  ros::spinOnce();
}

void CreateDriver::spin()
{
  ros::Rate rate(loop_hz_);
  while (ros::ok())
  {
    spinOnce();

    is_running_slowly_ = !rate.sleep();
    if (is_running_slowly_)
    {
      ROS_WARN("[CREATE] Loop running slowly.");
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ca_driver");
  ros::NodeHandle nh;

  CreateDriver create_driver(nh);

  try
  {
    create_driver.spin();
  }
  catch (std::runtime_error& ex)
  {
    ROS_FATAL_STREAM("[CREATE] Runtime error: " << ex.what());
    return 1;
  }
  return 0;
}
