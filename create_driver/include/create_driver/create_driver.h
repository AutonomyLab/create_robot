#ifndef CREATE_AUTONOMY_CREATE_DRIVER_H
#define CREATE_AUTONOMY_CREATE_DRIVER_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "create/create.h"

    
static const double COVARIANCE[36] = {1e-3, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                      0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                                      0.0, 0.0, 0.0, 0.0, 0.0, 1e3};

class CreateDriver {
  private:
    create::Create* robot;
    nav_msgs::Odometry odom;
    tf::TransformBroadcaster tfBroadcaster;
    geometry_msgs::TransformStamped tfOdom;
    ros::Time lastCmdVelTime;
    std_msgs::Empty emptyMsg;

    // ROS params
    double loopHz;
    std::string dev;
    int baud;
    double latchDuration;

    void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg);
    void debrisLEDCallback(const std_msgs::BoolConstPtr& msg);
    void spotLEDCallback(const std_msgs::BoolConstPtr& msg);
    void dockLEDCallback(const std_msgs::BoolConstPtr& msg);
    void checkLEDCallback(const std_msgs::BoolConstPtr& msg);
    void powerLEDCallback(const std_msgs::UInt8MultiArrayConstPtr& msg);
    void setASCIICallback(const std_msgs::UInt8MultiArrayConstPtr& msg);

    bool update();
    void publishOdom();
    void publishButtonPresses() const;

  protected:
    ros::NodeHandle nh;
    ros::NodeHandle privNh;
    ros::Subscriber cmdVelSub;
    ros::Subscriber debrisLEDSub;
    ros::Subscriber spotLEDSub;
    ros::Subscriber dockLEDSub;
    ros::Subscriber checkLEDSub;
    ros::Subscriber powerLEDSub;
    ros::Subscriber setASCIISub;
    ros::Publisher odomPub;
    ros::Publisher cleanBtnPub;
    ros::Publisher dayBtnPub;
    ros::Publisher hourBtnPub;
    ros::Publisher minBtnPub;
    ros::Publisher dockBtnPub;
    ros::Publisher spotBtnPub;

  public:
    CreateDriver(ros::NodeHandle& nh_);
    ~CreateDriver();
    virtual void spin();
    virtual void spinOnce();

}; // class CreateDriver

#endif // CREATE_AUTONOMY_CREATE_DRIVER_H
