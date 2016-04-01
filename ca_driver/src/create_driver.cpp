#include <tf/transform_datatypes.h>
#include "create_driver/create_driver.h"

CreateDriver::CreateDriver(ros::NodeHandle& nh) : nh_(nh), priv_nh_("~")
{
  bool create_one;
  priv_nh_.param<double>("loop_hz", loop_hz_, 10.0);
  priv_nh_.param<std::string>("dev", dev_, "/dev/ttyUSB0");
  priv_nh_.param<bool>("create_1", create_one, false);
  priv_nh_.param<double>("latch_cmd_duration", latch_duration_, 0.2);

  if (create_one)
  {
    model_ = create::CREATE_1;
    priv_nh_.param<int>("baud", baud_, 57600);
    ROS_INFO("[CREATE] Create 1 model selected");
  }
  else
  {
    model_ = create::CREATE_2;
    priv_nh_.param<int>("baud", baud_, 115200);
    ROS_INFO("[CREATE] Create 2 model selected");
  }

  robot_ = new create::Create(model_);

  if (!robot_->connect(dev_, baud_))
  {
    ROS_FATAL("[CREATE] Failed to establish serial connection with Create.");
    ros::shutdown();
  }

  ROS_INFO("[CREATE] Connection established.");

  // Put into full control mode
  // TODO: Make option to run in safe mode as parameter
  robot_->setMode(create::MODE_FULL);

  // Show robot's battery level
  ROS_INFO("[CREATE] Battery level %.2f %%", (robot_->getBatteryCharge() / (float)robot_->getBatteryCapacity()) * 100.0);

  // Set frame_id's
  const std::string str_base_footprint("base_footprint");
  mode_msg_.header.frame_id = str_base_footprint;
  bumper_msg_.header.frame_id = str_base_footprint;
  charging_state_msg_.header.frame_id = str_base_footprint;
  tf_odom_.header.frame_id = "odom";
  tf_odom_.child_frame_id = str_base_footprint;
  odom_msg_.header.frame_id = "odom";
  odom_msg_.child_frame_id = str_base_footprint;

  // Populate covariances
  for (int i = 0; i < 36; i++)
  {
    odom_msg_.pose.covariance[i] = COVARIANCE[i];
    odom_msg_.twist.covariance[i] = COVARIANCE[i];
  }

  cmd_vel_sub_ = nh.subscribe("cmd_vel", 1, &CreateDriver::cmdVelCallback, this);
  debris_led_sub_ = nh.subscribe("debris_led", 10, &CreateDriver::debrisLEDCallback, this);
  spot_led_sub_ = nh.subscribe("spot_led", 10, &CreateDriver::spotLEDCallback, this);
  dock_led_sub_ = nh.subscribe("dock_led", 10, &CreateDriver::dockLEDCallback, this);
  check_led_sub_ = nh.subscribe("check_led", 10, &CreateDriver::checkLEDCallback, this);
  power_led_sub_ = nh.subscribe("power_led", 10, &CreateDriver::powerLEDCallback, this);
  set_ascii_sub_ = nh.subscribe("set_ascii", 10, &CreateDriver::setASCIICallback, this);
  dock_sub_ = nh.subscribe("dock", 10, &CreateDriver::dockCallback, this);
  undock_sub_ = nh.subscribe("undock", 10, &CreateDriver::undockCallback, this);

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

  if (model_ == create::CREATE_1)
    usleep(1000000);  // Create 1 requires a delay (1 sec)

  // Call docking behaviour
  robot_->dock();
}

void CreateDriver::undockCallback(const std_msgs::EmptyConstPtr& msg)
{
  // Switch robot back to FULL mode
  robot_->setMode(create::MODE_FULL);
}

bool CreateDriver::update()
{
  publishOdom();
  publishBatteryInfo();
  publishButtonPresses();
  publishOmniChar();
  publishState();
  publishBumperInfo();
  publishWheeldrop();

  // If last velocity command was sent longer than latch duration, stop robot
  if (ros::Time::now() - last_cmd_vel_time_ >= ros::Duration(latch_duration_))
  {
    robot_->drive(0, 0);
  }

  return true;
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
  tf_odom_.header.stamp = ros::Time::now();
  tf_odom_.transform.translation.x = pose.x;
  tf_odom_.transform.translation.y = pose.y;
  tf_odom_.transform.rotation = quat;

  // Populate velocity info
  odom_msg_.twist.twist.linear.x = vel.x;
  odom_msg_.twist.twist.linear.y = vel.y;
  odom_msg_.twist.twist.angular.z = vel.yaw;

  // Update covariances
  if (fabs(vel.x) < 0.01 && fabs(vel.yaw) < 0.01)
  {
    odom_msg_.pose.covariance[0] = 1e-9;
    odom_msg_.pose.covariance[8] = 1e-9;
    odom_msg_.pose.covariance[35] = 1e-9;
    odom_msg_.twist.covariance[0] = 1e-9;
    odom_msg_.twist.covariance[8] = 1e-9;
    odom_msg_.twist.covariance[35] = 1e-9;
  }
  else
  {
    odom_msg_.pose.covariance[0] = 1e-3;
    odom_msg_.pose.covariance[8] = 0.0;
    odom_msg_.pose.covariance[35] = 1e3;
    odom_msg_.twist.covariance[0] = 1e-3;
    odom_msg_.twist.covariance[8] = 0.0;
    odom_msg_.twist.covariance[35] = 1e3;
  }

  tf_broadcaster_.sendTransform(tf_odom_);
  odom_pub_.publish(odom_msg_);
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
  float32_msg_.data = (float)robot_->getBatteryCharge() / (float)robot_->getBatteryCapacity();
  charge_ratio_pub_.publish(float32_msg_);
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
  // TODO: Publish info based on character, such as dock in sight
}

void CreateDriver::publishState()
{
  const create::CreateMode mode = robot_->getMode();
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
  }
  mode_pub_.publish(mode_msg_);

  const create::ChargingState charging_state = robot_->getChargingState();
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

void CreateDriver::publishBumperInfo()
{
  bumper_msg_.header.stamp = ros::Time::now();
  bumper_msg_.is_left_pressed = robot_->isLeftBumper();
  bumper_msg_.is_right_pressed = robot_->isRightBumper();

  if (model_ == create::CREATE_2)
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
  ros::spinOnce();
}

void CreateDriver::spin()
{
  ros::Rate rate(loop_hz_);
  while (ros::ok())
  {
    spinOnce();
    if (!rate.sleep())
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
