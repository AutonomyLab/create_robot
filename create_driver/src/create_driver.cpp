#include <tf/transform_datatypes.h>
#include "create_driver/create_driver.h"

CreateDriver::CreateDriver(ros::NodeHandle& nh_) : nh(nh_) {
  priv_nh.param<double>("loop_hz", loopHz, 10);
  priv_nh.param<std::string>("dev", dev, "/dev/ttyUSB0");
  priv_nh.param<int>("baud", baud, 115200);
  priv_nh.param<double>("latch_cmd_duration", latchDuration, 0.5);

  robot = new create::Create();

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

  // Not sure if these are correct
  odom.header.frame_id = "base_link";
  odom.child_frame_id = "odom";

  cmdVelSub = nh.subscribe(std::string("cmd_vel"), 1, &CreateDriver::cmdVelCallback, this);

  odomPub = nh.advertise<nav_msgs::Odometry>(std::string("odom"), 10);
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

bool CreateDriver::update() {
  publishOdom();

  // If last velocity command was sent longer than latch duration, stop robot
  if (ros::Time::now() - lastCmdVelTime >= ros::Duration(latchDuration)) {
    robot->drive(0, 0);
  }

  return true;
}

void CreateDriver::publishOdom() {
  create::Pose pose = robot->getPose();
  odom.pose.pose.position.x = pose.x;
  odom.pose.pose.position.y = pose.y;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose.yaw);

  // TODO: populate pose covariance
  //odom.pose.covariance = ?

  // TODO: populate twist data
  //odom.twist.twist.linear.x = ?
  //odom.twist.twist.angular.z = ?
  //odom.twist.covariance = ?
  odomPub.publish(odom);
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
