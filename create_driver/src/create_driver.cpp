#include <tf/transform_datatypes.h>
#include "create_driver/create_driver.h"

CreateDriver::CreateDriver(ros::NodeHandle& nh_) : nh(nh_), privNh("~") {
  bool createOne;
  privNh.param<double>("loop_hz", loopHz, 10);
  privNh.param<std::string>("dev", dev, "/dev/ttyUSB0");
  privNh.param<bool>("create_1", createOne, false);
  privNh.param<double>("latch_cmd_duration", latchDuration, 0.2);

  if (createOne) {
    model = create::CREATE_1;
    privNh.param<int>("baud", baud, 57600);
    ROS_INFO("[CREATE] Create 1 model selected");
  }
  else {
    model = create::CREATE_2;
    privNh.param<int>("baud", baud, 115200);
    ROS_INFO("[CREATE] Create 2 model selected");
  }

  robot = new create::Create(model);

  if (!robot->connect(dev, baud)) {
    ROS_FATAL("[CREATE] Failed to establish serial connection with Create.");
    ros::shutdown();
  }

  ROS_INFO("[CREATE] Connection established.");

  // Put into full control mode
  //TODO: Make option to run in safe mode as parameter
  robot->setMode(create::MODE_FULL);

  // Show robot's battery level
  ROS_INFO("[CREATE] Battery level %.2f %%", (robot->getBatteryCharge() / (float)robot->getBatteryCapacity()) * 100.0);

  tfOdom.header.frame_id = "odom";
  tfOdom.child_frame_id = "base_footprint";
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  // Populate covariances
  for (int i = 0; i < 36; i++) {
    odom.pose.covariance[i] = COVARIANCE[i];
    odom.twist.covariance[i] = COVARIANCE[i];
  }

  cmdVelSub = nh.subscribe("cmd_vel", 1, &CreateDriver::cmdVelCallback, this);
  debrisLEDSub = nh.subscribe("debris_led", 10, &CreateDriver::debrisLEDCallback, this);
  spotLEDSub = nh.subscribe("spot_led", 10, &CreateDriver::spotLEDCallback, this);
  dockLEDSub = nh.subscribe("dock_led", 10, &CreateDriver::dockLEDCallback, this);
  checkLEDSub = nh.subscribe("check_led", 10, &CreateDriver::checkLEDCallback, this);
  powerLEDSub = nh.subscribe("power_led", 10, &CreateDriver::powerLEDCallback, this);
  setASCIISub = nh.subscribe("set_ascii", 10, &CreateDriver::setASCIICallback, this);
  dockSub = nh.subscribe("dock", 10, &CreateDriver::dockCallback, this);
  undockSub = nh.subscribe("undock", 10, &CreateDriver::undockCallback, this);

  odomPub = nh.advertise<nav_msgs::Odometry>("odom", 30);
  cleanBtnPub = nh.advertise<std_msgs::Empty>("clean_button", 30);
  dayBtnPub = nh.advertise<std_msgs::Empty>("day_button", 30);
  hourBtnPub = nh.advertise<std_msgs::Empty>("hour_button", 30);
  minBtnPub = nh.advertise<std_msgs::Empty>("minute_button",30);
  dockBtnPub = nh.advertise<std_msgs::Empty>("dock_button", 30);
  spotBtnPub = nh.advertise<std_msgs::Empty>("spot_button", 30);
  voltagePub = nh.advertise<std_msgs::UInt16>("battery/voltage", 30);
  currentPub = nh.advertise<std_msgs::UInt16>("battery/current", 30);
  chargePub = nh.advertise<std_msgs::UInt16>("battery/charge", 30);
  capacityPub = nh.advertise<std_msgs::UInt16>("battery/capacity", 30);
  temperaturePub = nh.advertise<std_msgs::UInt16>("battery/temperature", 30);
  omniCharPub = nh.advertise<std_msgs::UInt16>("ir_omni", 30);
  ROS_INFO("[CREATE] Ready.");
}

CreateDriver::~CreateDriver() {
  ROS_INFO("[CREATE] Destruct sequence initiated.");
  robot->disconnect();
  delete robot;
}

void CreateDriver::cmdVelCallback(const geometry_msgs::TwistConstPtr& msg) {
  robot->drive(msg->linear.x, msg->angular.z);
  lastCmdVelTime = ros::Time::now();
}

void CreateDriver::debrisLEDCallback(const std_msgs::BoolConstPtr& msg) {
  robot->enableDebrisLED(msg->data);
}

void CreateDriver::spotLEDCallback(const std_msgs::BoolConstPtr& msg) {
  robot->enableSpotLED(msg->data);
}

void CreateDriver::dockLEDCallback(const std_msgs::BoolConstPtr& msg) {
  robot->enableDockLED(msg->data);
}

void CreateDriver::checkLEDCallback(const std_msgs::BoolConstPtr& msg) {
  robot->enableCheckRobotLED(msg->data);
}

void CreateDriver::powerLEDCallback(const std_msgs::UInt8MultiArrayConstPtr& msg) {
  if (msg->data.size() < 1) {
    ROS_ERROR("[CREATE] No values provided to set power LED");
  }
  else {
    if (msg->data.size() < 2) {
      robot->setPowerLED(msg->data[0]);
    }
    else {
      robot->setPowerLED(msg->data[0], msg->data[1]);
    }
  }
}

void CreateDriver::setASCIICallback(const std_msgs::UInt8MultiArrayConstPtr& msg) {
  bool result;
  if (msg->data.size() < 1) {
    ROS_ERROR("[CREATE] No ASCII digits provided");
  }
  else if (msg->data.size() < 2) {
    result = robot->setDigitsASCII(msg->data[0], ' ', ' ', ' ');
  }
  else if (msg->data.size() < 3) {
    result = robot->setDigitsASCII(msg->data[0], msg->data[1], ' ', ' ');
  }
  else if (msg->data.size() < 4) {
    result = robot->setDigitsASCII(msg->data[0], msg->data[1], msg->data[2], ' ');
  }
  else {
    result = robot->setDigitsASCII(msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
  }

  if (!result) {
    ROS_ERROR("[CREATE] ASCII character out of range [32, 126]");
  }
}

void CreateDriver::dockCallback(const std_msgs::EmptyConstPtr& msg) {
  // Call docking behaviour
  robot->dock();
}

void CreateDriver::undockCallback(const std_msgs::EmptyConstPtr& msg) {
  // Switch robot back to FULL mode
  robot->setMode(create::MODE_FULL);
}

bool CreateDriver::update() {
  publishOdom();
  publishBatteryInfo();
  publishButtonPresses();
  publishOmniChar();

  // If last velocity command was sent longer than latch duration, stop robot
  if (ros::Time::now() - lastCmdVelTime >= ros::Duration(latchDuration)) {
    robot->drive(0, 0);
  }

  return true;
}

void CreateDriver::publishOdom() {
  create::Pose pose = robot->getPose();
  create::Vel vel = robot->getVel();

  // Populate position info
  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose.yaw);
  odom.pose.pose.position.x = pose.x;
  odom.pose.pose.position.y = pose.y;
  odom.pose.pose.orientation = quat;
  tfOdom.header.stamp = ros::Time::now();
  tfOdom.transform.translation.x = pose.x;
  tfOdom.transform.translation.y = pose.y;
  tfOdom.transform.rotation = quat;

  // Populate velocity info
  odom.twist.twist.linear.x = vel.x;
  odom.twist.twist.linear.y = vel.y;
  odom.twist.twist.angular.z = vel.yaw;

  // Update covariances
  if (fabs(vel.x) < 0.01 && fabs(vel.yaw) < 0.01) {
    odom.pose.covariance[0] = 1e-9;
    odom.pose.covariance[8] = 1e-9;
    odom.pose.covariance[35] = 1e-9;
    odom.twist.covariance[0] = 1e-9;
    odom.twist.covariance[8] = 1e-9;
    odom.twist.covariance[35] = 1e-9;
  }
  else {
    odom.pose.covariance[0] = 1e-3;
    odom.pose.covariance[8] = 0.0;
    odom.pose.covariance[35] = 1e3;
    odom.twist.covariance[0] = 1e-3;
    odom.twist.covariance[8] = 0.0;
    odom.twist.covariance[35] = 1e3;
  }

  tfBroadcaster.sendTransform(tfOdom);
  odomPub.publish(odom);
}

void CreateDriver::publishBatteryInfo() {
  uint16Msg.data = robot->getVoltage();
  voltagePub.publish(uint16Msg);
  uint16Msg.data = robot->getCurrent();
  currentPub.publish(uint16Msg);
  uint16Msg.data = robot->getBatteryCharge();
  chargePub.publish(uint16Msg);
  uint16Msg.data = robot->getBatteryCapacity();
  capacityPub.publish(uint16Msg);
  uint16Msg.data = robot->getTemperature();
  temperaturePub.publish(uint16Msg);
}

void CreateDriver::publishButtonPresses() const {
  if (robot->isCleanButtonPressed()) {
    cleanBtnPub.publish(emptyMsg);
  }
  if (robot->isDayButtonPressed()) {
    dayBtnPub.publish(emptyMsg);
  }
  if (robot->isHourButtonPressed()) {
    hourBtnPub.publish(emptyMsg);
  }
  if (robot->isMinButtonPressed()) {
    minBtnPub.publish(emptyMsg);
  }
  if (robot->isDockButtonPressed()) {
    dockBtnPub.publish(emptyMsg);
  }
  if (robot->isSpotButtonPressed()) {
    spotBtnPub.publish(emptyMsg);
  }
}

void CreateDriver::publishOmniChar() {
  uint8_t irChar = robot->getIROmni();
  uint16Msg.data = irChar;
  omniCharPub.publish(uint16Msg);
  // TODO: Publish info based on character, such as dock in sight
}

void CreateDriver::spinOnce() {
  update();
  ros::spinOnce();
}

void CreateDriver::spin() {
  ros::Rate rate(loopHz);
  while (ros::ok()) {
    spinOnce();
    if (!rate.sleep()) {
      ROS_WARN("[CREATE] Loop running slowly.");
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "create_driver");
  ros::NodeHandle nh;

  CreateDriver createDriver(nh);

  try {
    createDriver.spin();
  }
  catch (std::runtime_error& ex) {
    ROS_FATAL_STREAM("[CREATE] Runtime error: " << ex.what());
    return 1;
  }
  return 0;
}
