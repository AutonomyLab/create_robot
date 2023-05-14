/**
Software License Agreement (BSD)
\file      create_driver.h
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

#ifndef CREATE_DRIVER__CREATE_DRIVER_H_
#define CREATE_DRIVER__CREATE_DRIVER_H_
#include <string>

#include "create_msgs/msg/charging_state.hpp"
#include "create_msgs/msg/mode.hpp"
#include "create_msgs/msg/bumper.hpp"
#include "create_msgs/msg/cliff.hpp"
#include "create_msgs/msg/define_song.hpp"
#include "create_msgs/msg/play_song.hpp"
#include "create_msgs/msg/motor_setpoint.hpp"

#include "create/create.h"

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"

static const double COVARIANCE[36] = {1e-5, 1e-5, 0.0,  0.0,  0.0,  1e-5,  // NOLINT(whitespace/braces)
                                      1e-5, 1e-5, 0.0,  0.0,  0.0,  1e-5,
                                      0.0,  0.0,  1e-5, 0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.0,  1e-5, 0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.0,  1e-5, 0.0,
                                      1e-5, 1e-5, 0.0,  0.0,  0.0,  1e-5};

class CreateDriver : public rclcpp::Node
{
private:
  create::Create* robot_;
  create::RobotModel model_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr debris_led_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr spot_led_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dock_led_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr check_led_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr power_led_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr set_ascii_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr dock_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr undock_sub_;
  rclcpp::Subscription<create_msgs::msg::DefineSong>::SharedPtr define_song_sub_;
  rclcpp::Subscription<create_msgs::msg::PlaySong>::SharedPtr play_song_sub_;
  rclcpp::Subscription<create_msgs::msg::MotorSetpoint>::SharedPtr side_brush_motor_sub_;
  rclcpp::Subscription<create_msgs::msg::MotorSetpoint>::SharedPtr main_brush_motor_sub_;
  rclcpp::Subscription<create_msgs::msg::MotorSetpoint>::SharedPtr vacuum_motor_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr clean_btn_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr day_btn_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr hour_btn_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr min_btn_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr dock_btn_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr spot_btn_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr charge_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr charge_ratio_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr capacity_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr temperature_pub_;
  rclcpp::Publisher<create_msgs::msg::ChargingState>::SharedPtr charging_state_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr omni_char_pub_;
  rclcpp::Publisher<create_msgs::msg::Mode>::SharedPtr mode_pub_;
  rclcpp::Publisher<create_msgs::msg::Bumper>::SharedPtr bumper_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr wheeldrop_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_joint_pub_;
  rclcpp::Publisher<create_msgs::msg::Cliff>::SharedPtr cliff_pub_;

  rclcpp::TimerBase::SharedPtr loop_timer_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  diagnostic_updater::Updater diagnostics_;

  create_msgs::msg::Mode mode_msg_;
  create_msgs::msg::ChargingState charging_state_msg_;
  create_msgs::msg::Bumper bumper_msg_;
  create_msgs::msg::Cliff cliff_msg_;
  nav_msgs::msg::Odometry odom_msg_;
  geometry_msgs::msg::TransformStamped tf_odom_;
  rclcpp::Time last_cmd_vel_time_;
  std_msgs::msg::Empty empty_msg_;
  std_msgs::msg::Float32 float32_msg_;
  std_msgs::msg::UInt16 uint16_msg_;
  std_msgs::msg::Int16 int16_msg_;
  sensor_msgs::msg::JointState joint_state_msg_;
  bool is_running_slowly_;

  // ROS params
  std::string dev_;
  std::string base_frame_;
  std::string odom_frame_;
  rclcpp::Duration latch_duration_;
  double loop_hz_;
  bool publish_tf_;
  int baud_;
  bool oi_mode_workaround_;

  void cmdVelCallback(geometry_msgs::msg::Twist::UniquePtr msg);
  void debrisLEDCallback(std_msgs::msg::Bool::UniquePtr msg);
  void spotLEDCallback(std_msgs::msg::Bool::UniquePtr msg);
  void dockLEDCallback(std_msgs::msg::Bool::UniquePtr msg);
  void checkLEDCallback(std_msgs::msg::Bool::UniquePtr msg);
  void powerLEDCallback(std_msgs::msg::UInt8MultiArray::UniquePtr msg);
  void setASCIICallback(std_msgs::msg::UInt8MultiArray::UniquePtr msg);
  void dockCallback(std_msgs::msg::Empty::UniquePtr msg);
  void undockCallback(std_msgs::msg::Empty::UniquePtr msg);
  void defineSongCallback(create_msgs::msg::DefineSong::UniquePtr msg);
  void playSongCallback(create_msgs::msg::PlaySong::UniquePtr msg);
  void sideBrushMotor(create_msgs::msg::MotorSetpoint::UniquePtr msg);
  void mainBrushMotor(create_msgs::msg::MotorSetpoint::UniquePtr msg);
  void vacuumBrushMotor(create_msgs::msg::MotorSetpoint::UniquePtr msg);

  bool update();
  void updateBatteryDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void updateSafetyDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void updateSerialDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void updateModeDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void updateDriverDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void publishOdom();
  void publishJointState();
  void publishBatteryInfo();
  void publishButtonPresses() const;
  void publishOmniChar();
  void publishMode();
  void publishBumperInfo();
  void publishWheeldrop();
  void publishCliff();

public:
  CreateDriver();
  ~CreateDriver();
  virtual void spinOnce();
};  // class CreateDriver

#endif  // CREATE_DRIVER__CREATE_DRIVER_H_
