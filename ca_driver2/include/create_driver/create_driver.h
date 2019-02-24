#pragma once

#include <limits>
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
//#include <diagnostic_updater/diagnostic_updater.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "create/create.h"
#include "ca_msgs/msg/charging_state.hpp"
#include "ca_msgs/msg/mode.hpp"
#include "ca_msgs/msg/bumper.hpp"
#include "ca_msgs/msg/define_song.hpp"
#include "ca_msgs/msg/play_song.hpp"

static const double MAX_DBL = std::numeric_limits<double>::max();
static const double COVARIANCE[36] = {1e-5, 1e-5, 0.0,     0.0,     0.0,     1e-5,  // NOLINT(whitespace/braces)
                                      1e-5, 1e-5, 0.0,     0.0,     0.0,     1e-5,
                                      0.0,  0.0,  MAX_DBL, 0.0,     0.0,     0.0,
                                      0.0,  0.0,  0.0,     MAX_DBL, 0.0,     0.0,
                                      0.0,  0.0,  0.0,     0.0,     MAX_DBL, 0.0,
                                      1e-5, 1e-5, 0.0,     0.0,     0.0,     1e-5};

class CreateDriver : public rclcpp::Node
{
private:
  std::unique_ptr<create::Create> robot_;
  create::RobotModel model_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
//  diagnostic_updater::Updater diagnostics_;
  ca_msgs::msg::Mode mode_msg_;
  ca_msgs::msg::ChargingState charging_state_msg_;
  ca_msgs::msg::Bumper bumper_msg_;
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
  double latch_duration_;
  double loop_hz_;
  bool publish_tf_;
  int baud_;

  void cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr & msg);
  void debrisLEDCallback(const std_msgs::msg::Bool::ConstSharedPtr & msg);
  void spotLEDCallback(const std_msgs::msg::Bool::ConstSharedPtr & msg);
  void dockLEDCallback(const std_msgs::msg::Bool::ConstSharedPtr & msg);
  void checkLEDCallback(const std_msgs::msg::Bool::ConstSharedPtr & msg);
  void powerLEDCallback(const std_msgs::msg::UInt8MultiArray::ConstSharedPtr & msg);
  void setASCIICallback(const std_msgs::msg::UInt8MultiArray::ConstSharedPtr & msg);
  void dockCallback(const std_msgs::msg::Empty::ConstSharedPtr & msg);
  void undockCallback(const std_msgs::msg::Empty::ConstSharedPtr & msg);
  void defineSongCallback(const ca_msgs::msg::DefineSong::ConstSharedPtr & msg);
  void playSongCallback(const ca_msgs::msg::PlaySong::ConstSharedPtr & msg);

  bool update();
//  void updateBatteryDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
//  void updateSafetyDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
//  void updateSerialDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
//  void updateModeDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
//  void updateDriverDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void publishOdom();
  void publishJointState();
  void publishBatteryInfo();
  void publishButtonPresses() const;
  void publishOmniChar();
  void publishMode();
  void publishBumperInfo();
  void publishWheeldrop();

protected:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr debris_led_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr spot_led_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dock_led_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr check_led_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr power_led_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr set_ascii_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr dock_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr undock_sub_;
  rclcpp::Subscription<ca_msgs::msg::DefineSong>::SharedPtr define_song_sub_;
  rclcpp::Subscription<ca_msgs::msg::PlaySong>::SharedPtr play_song_sub_;

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
  rclcpp::Publisher<ca_msgs::msg::ChargingState>::SharedPtr charging_state_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr omni_char_pub_;
  rclcpp::Publisher<ca_msgs::msg::Mode>::SharedPtr mode_pub_;
  rclcpp::Publisher<ca_msgs::msg::Bumper>::SharedPtr bumper_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr wheeldrop_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_joint_pub_;

public:
  explicit CreateDriver(const std::string & name);
  ~CreateDriver();
};  // class CreateDriver
