#ifndef CREATE_AUTONOMY_CREATE_DRIVER_H
#define CREATE_AUTONOMY_CREATE_DRIVER_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include "create/create.h"
#include "ca_msgs/ChargingState.h"
#include "ca_msgs/Mode.h"
#include "ca_msgs/Bumper.h"
#include "ca_msgs/Wheeldrop.h"
#include "ca_msgs/Cliff.h"


namespace create
{

static const uint8_t SONG_0_LENGTH = 2;
static const uint8_t SONG_0_NOTES [] = {64,54};
static const float SONG_0_DURATIONS [] = {1.0,1.0};

static const uint8_t SONG_1_LENGTH = 16;
static const uint8_t SONG_1_NOTES [] = {105,103,100,96,98,107,101,108,105,103,100,96,98,107,103,108};
static const float SONG_1_DURATIONS [] = {0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1};

static const uint8_t SONG_2_LENGTH = 16;
static const uint8_t SONG_2_NOTES [] = {84,107,84,107,84,107,84,107,84,107,84,107,84,107,84,107};
static const float SONG_2_DURATIONS [] = {0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1};

static const uint8_t SONG_3_LENGTH = 11;
static const uint8_t SONG_3_NOTES [] = {59,59,59,59,62,61,61,59,59,59,59};
static const float SONG_3_DURATIONS [] = {1.0,0.75,0.25,1.0,0.75,0.25,0.7,0.25,0.7,0.25,1.0};

static const uint8_t SONG_4_LENGTH = 16;
static const uint8_t SONG_4_NOTES [] = {79,86,84,83,81,91,86,84,83,81,91,86,84,83,84,81};
static const float SONG_4_DURATIONS [] = {0.9,0.8,0.2,0.2,0.2,0.8,0.7,0.2,0.2,0.2,0.8,0.7,0.2,0.2,0.2,0.9};


static const double MAX_DBL = std::numeric_limits<double>::max();
static const double COVARIANCE[36] = {1e-5, 1e-5, 0.0,     0.0,     0.0,     1e-5,
                                      1e-5, 1e-5, 0.0,     0.0,     0.0,     1e-5,
                                      0.0,  0.0,  MAX_DBL, 0.0,     0.0,     0.0,
                                      0.0,  0.0,  0.0,     MAX_DBL, 0.0,     0.0,
                                      0.0,  0.0,  0.0,     0.0,     MAX_DBL, 0.0,
                                      1e-5, 1e-5, 0.0,     0.0,     0.0,     1e-5};

class CreateDriver
{
private:
  create::Create* robot_;
  create::RobotModel model_;
  tf::TransformBroadcaster tf_broadcaster_;
  diagnostic_updater::Updater diagnostics_;
  ca_msgs::Mode mode_msg_;
  ca_msgs::ChargingState charging_state_msg_;
  ca_msgs::Bumper bumper_msg_;
  ca_msgs::Cliff cliff_msg_;
  ca_msgs::Wheeldrop wheeldrop_msg_;
  nav_msgs::Odometry odom_msg_;
  geometry_msgs::TransformStamped tf_odom_;
  ros::Time last_cmd_vel_time_;
  std_msgs::Empty empty_msg_;
  std_msgs::Float32 float32_msg_;
  std_msgs::UInt16 uint16_msg_;
  std_msgs::Int16 int16_msg_;
  sensor_msgs::JointState joint_state_msg_;
  bool is_running_slowly_;

  // ROS params
  double loop_hz_;
  std::string dev_;
  int baud_;
  double latch_duration_;
  bool publish_tf_;

  void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg);
  void debrisLEDCallback(const std_msgs::BoolConstPtr& msg);
  void spotLEDCallback(const std_msgs::BoolConstPtr& msg);
  void dockLEDCallback(const std_msgs::BoolConstPtr& msg);
  void checkLEDCallback(const std_msgs::BoolConstPtr& msg);
  void powerLEDCallback(const std_msgs::UInt8MultiArrayConstPtr& msg);
  void setASCIICallback(const std_msgs::UInt8MultiArrayConstPtr& msg);
  void playSongCallback(const std_msgs::UInt8ConstPtr& msg);
  void dockCallback(const std_msgs::EmptyConstPtr& msg);
  void undockCallback(const std_msgs::EmptyConstPtr& msg);
  void mainMotorCallback(const std_msgs::Float32ConstPtr& msg);

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
  void publishCliffInfo();
  void publishWheeldrop();

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber debris_led_sub_;
  ros::Subscriber spot_led_sub_;
  ros::Subscriber dock_led_sub_;
  ros::Subscriber check_led_sub_;
  ros::Subscriber power_led_sub_;
  ros::Subscriber set_ascii_sub_;
  ros::Subscriber play_song_sub_;
  ros::Subscriber dock_sub_;
  ros::Subscriber undock_sub_;
  ros::Subscriber main_motor_sub_;

  ros::Publisher odom_pub_;
  ros::Publisher clean_btn_pub_;
  ros::Publisher day_btn_pub_;
  ros::Publisher hour_btn_pub_;
  ros::Publisher min_btn_pub_;
  ros::Publisher dock_btn_pub_;
  ros::Publisher spot_btn_pub_;
  ros::Publisher voltage_pub_;
  ros::Publisher current_pub_;
  ros::Publisher charge_pub_;
  ros::Publisher charge_ratio_pub_;
  ros::Publisher capacity_pub_;
  ros::Publisher temperature_pub_;
  ros::Publisher charging_state_pub_;
  ros::Publisher omni_char_pub_;
  ros::Publisher mode_pub_;
  ros::Publisher bumper_pub_;
  ros::Publisher cliff_pub_;
  ros::Publisher wheeldrop_pub_;
  ros::Publisher wheel_joint_pub_;

public:
  CreateDriver(ros::NodeHandle& nh, ros::NodeHandle& ph);
  ~CreateDriver();

  virtual void spin();
  virtual void spinOnce();

};  // class CreateDriver

} //namespace ccreate

#endif  // CREATE_AUTONOMY_CREATE_DRIVER_H
