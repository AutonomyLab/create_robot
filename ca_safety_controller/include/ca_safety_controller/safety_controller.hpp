/*
 * Copyright (c) 2012, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file /ca_safety_controller/include/ca_safety_controller/safety_controller.hpp
 *
 * @brief Create-specific safety controller
 *
 * This controller uses Create's bumper, cliff and wheel drop sensors to ensure safe operation.
 *
 * @author Marcus Liebhardt, Yujin Robot
 *
 **/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef SAFETY_CONTROLLER_HPP_
#define SAFETY_CONTROLLER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <ros/ros.h>
#include <yocs_controllers/default_controller.hpp>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ca_msgs/Bumper.h>
#include <ca_msgs/Cliff.h>
#include <ca_msgs/Wheeldrop.h>

namespace ca
{

/**
 * @ brief Keeps track of safety-related events and commands Create to move accordingly
 *
 * The SafetyController keeps track of bumper, cliff and wheel drop events. In case of the first two,
 * Create is commanded to move back. In the latter case, Create is stopped. All commands stop when the
 * event condition disappears. In the case of lateral bump/cliff, robot also spins a bit, what makes
 * easier to escape from the risk.
 *
 * This controller can be enabled/disabled.
 * The safety states (bumper pressed etc.) can be reset. WARNING: Dangerous!
 */
class SafetyController : public yocs::Controller
{
public:
  SafetyController(ros::NodeHandle& nh, std::string& name) :
    Controller(),
    nh_(nh),
    name_(name),
    wheel_left_dropped_(false),
    wheel_right_dropped_(false),
    bumper_left_pressed_(false),
    bumper_center_pressed_(false),
    bumper_right_pressed_(false),
    cliff_left_detected_(false),
    cliff_center_detected_(false),
    cliff_right_detected_(false), 
    last_event_time_(ros::Time(0)),
    msg_(new geometry_msgs::Twist()){};
  ~SafetyController(){};

  /**
   * Set-up necessary publishers/subscribers and variables
   * @return true, if successful
   */
  bool init()
  {
    //how long to keep sending messages after a bump, cliff, or wheel drop stops
    double time_to_extend_bump_cliff_events;
    nh_.param("time_to_extend_bump_cliff_events", time_to_extend_bump_cliff_events, 0.0);
    time_to_extend_bump_cliff_events_ = ros::Duration(time_to_extend_bump_cliff_events);
    enable_controller_subscriber_ = nh_.subscribe("enable", 10, &SafetyController::enableCB, this);
    disable_controller_subscriber_ = nh_.subscribe("disable", 10, &SafetyController::disableCB, this);
    bumper_event_subscriber_ = nh_.subscribe("bumper", 10, &SafetyController::bumperEventCB, this);
    cliff_event_subscriber_  = nh_.subscribe("cliff",  10, &SafetyController::cliffEventCB, this);
    wheel_event_subscriber_  = nh_.subscribe("wheeldrop", 10, &SafetyController::wheelEventCB, this);
    reset_safety_states_subscriber_ = nh_.subscribe("reset", 10, &SafetyController::resetSafetyStatesCB, this);
    velocity_command_publisher_ = nh_.advertise< geometry_msgs::Twist >("cmd_vel", 10);
    return true;
  };

  /**
   * @ brief Checks safety states and publishes velocity commands when necessary
   */
  void spin();

private:
  ros::NodeHandle nh_;
  std::string name_;
  ros::Subscriber enable_controller_subscriber_, disable_controller_subscriber_;
  ros::Subscriber bumper_event_subscriber_, cliff_event_subscriber_, wheel_event_subscriber_;
  ros::Subscriber reset_safety_states_subscriber_;
  ros::Publisher controller_state_publisher_, velocity_command_publisher_;
  bool wheel_left_dropped_, wheel_right_dropped_;
  bool bumper_left_pressed_, bumper_center_pressed_, bumper_right_pressed_;
  bool cliff_left_detected_, cliff_center_detected_, cliff_right_detected_;
  ros::Duration time_to_extend_bump_cliff_events_;
  ros::Time last_event_time_;

  geometry_msgs::TwistPtr msg_; // velocity command

  /**
   * @brief ROS logging output for enabling the controller
   * @param msg incoming topic message
   */
  void enableCB(const std_msgs::EmptyConstPtr msg);

  /**
   * @brief ROS logging output for disabling the controller
   * @param msg incoming topic message
   */
  void disableCB(const std_msgs::EmptyConstPtr msg);

  /**
   * @brief Keeps track of bumps
   * @param msg incoming topic message
   */
  void bumperEventCB(const ca_msgs::BumperConstPtr msg);

  /**
   * @brief Keeps track of cliff detection
   * @param msg incoming topic message
   */
  void cliffEventCB(const ca_msgs::CliffConstPtr msg);

  /**
   * @brief Keeps track of the wheel states
   * @param msg incoming topic message
   */
  void wheelEventCB(const ca_msgs::WheeldropConstPtr msg);

  /**
   * @brief Callback for resetting safety variables
   *
   * Allows resetting bumper, cliff and wheel drop states.
   * DANGEROUS!
   *
   * @param msg incoming topic message
   */
  void resetSafetyStatesCB(const std_msgs::EmptyConstPtr msg);
};


void SafetyController::enableCB(const std_msgs::EmptyConstPtr msg)
{
  if (this->enable())
  {
    ROS_INFO_STREAM("Controller has been enabled. [" << name_ << "]");
  }
  else
  {
    ROS_INFO_STREAM("Controller was already enabled. [" << name_ <<"]");
  }
};

void SafetyController::disableCB(const std_msgs::EmptyConstPtr msg)
{
  if (this->disable())
  {
    ROS_INFO_STREAM("Controller has been disabled. [" << name_ <<"]");
  }
  else
  {
    ROS_INFO_STREAM("Controller was already disabled. [" << name_ <<"]");
  }
};

void SafetyController::cliffEventCB(const ca_msgs::CliffConstPtr msg)
{
  if (msg->is_cliff_left || msg->is_cliff_front_left || msg->is_cliff_front_right || msg->is_cliff_right)
  {
    last_event_time_ = ros::Time::now();
    ROS_DEBUG_STREAM("Cliff detected. Moving backwards. [" << name_ << "]");

    cliff_left_detected_   = msg->is_cliff_left;
    cliff_center_detected_ = msg->is_cliff_front_left && msg->is_cliff_front_right;
    cliff_right_detected_  = msg->is_cliff_right;

  }

  else // ca_msgs::CliffEvent::FLOOR
  {
    ROS_DEBUG_STREAM("Not detecting any cliffs. Resuming normal operation. [" << name_ << "]");

    cliff_left_detected_   = msg->is_cliff_left;
    cliff_center_detected_ = msg->is_cliff_front_left && msg->is_cliff_front_right;
    cliff_right_detected_  = msg->is_cliff_right;
  }
};

void SafetyController::bumperEventCB(const ca_msgs::BumperConstPtr msg)
{
  if (msg->is_left_pressed || msg->is_right_pressed)
  {
    last_event_time_ = ros::Time::now();
    ROS_DEBUG_STREAM("Bumper pressed. Moving backwards. [" << name_ << "]");

    bumper_left_pressed_   = msg->is_left_pressed;;
    bumper_center_pressed_ = msg->is_left_pressed && msg->is_right_pressed;
    bumper_right_pressed_  = msg->is_right_pressed;

  }
  else // ca_msgs::BumperEvent::RELEASED
  {
    ROS_DEBUG_STREAM("No bumper pressed. Resuming normal operation. [" << name_ << "]");

    bumper_left_pressed_   = msg->is_left_pressed;;
    bumper_center_pressed_ = msg->is_left_pressed && msg->is_right_pressed;
    bumper_right_pressed_  = msg->is_right_pressed;
  }
};

void SafetyController::wheelEventCB(const ca_msgs::WheeldropConstPtr msg)
{
  if (msg->is_left_dropped || msg->is_right_dropped)
  {
    ROS_DEBUG_STREAM("Wheel drop detected. Stoping base. [" << name_ << "]");

    wheel_left_dropped_ = msg->is_left_dropped;
    wheel_right_dropped_ = msg->is_right_dropped;

  }

  else // ca_msgs::WheelDropEvent::RAISED
  {
    ROS_DEBUG_STREAM("Both wheels raised. Resuming normal operation. [" << name_ << "]");
    wheel_left_dropped_ = msg->is_left_dropped;
    wheel_right_dropped_ = msg->is_right_dropped;

  }
};

void SafetyController::resetSafetyStatesCB(const std_msgs::EmptyConstPtr msg)
{
  wheel_left_dropped_    = false;
  wheel_right_dropped_   = false;
  bumper_left_pressed_   = false;
  bumper_center_pressed_ = false;
  bumper_right_pressed_  = false;
  cliff_left_detected_   = false;
  cliff_center_detected_ = false;
  cliff_right_detected_  = false;
  ROS_WARN_STREAM("All safety states have been reset to false. [" << name_ << "]");
}

void SafetyController::spin()
{
  if (this->getState())
  {
    if (wheel_left_dropped_ || wheel_right_dropped_)
    {
      msg_.reset(new geometry_msgs::Twist());
      msg_->linear.x = 0.0;
      msg_->linear.y = 0.0;
      msg_->linear.z = 0.0;
      msg_->angular.x = 0.0;
      msg_->angular.y = 0.0;
      msg_->angular.z = 0.0;
      velocity_command_publisher_.publish(msg_);
    }
    else if (bumper_center_pressed_ || cliff_center_detected_)
    {
      msg_.reset(new geometry_msgs::Twist());
      msg_->linear.x = -0.1;
      msg_->linear.y = 0.0;
      msg_->linear.z = 0.0;
      msg_->angular.x = 0.0;
      msg_->angular.y = 0.0;
      msg_->angular.z = 0.0;
      velocity_command_publisher_.publish(msg_);
    }
    else if (bumper_left_pressed_ || cliff_left_detected_)
    {
      // left bump/cliff; also spin a bit to the right to make escape easier
      msg_.reset(new geometry_msgs::Twist());
      msg_->linear.x = -0.1;
      msg_->linear.y = 0.0;
      msg_->linear.z = 0.0;
      msg_->angular.x = 0.0;
      msg_->angular.y = 0.0;
      msg_->angular.z = -0.4;
      velocity_command_publisher_.publish(msg_);
    }
    else if (bumper_right_pressed_ || cliff_right_detected_)
    {
      // right bump/cliff; also spin a bit to the left to make escape easier
      msg_.reset(new geometry_msgs::Twist());
      msg_->linear.x = -0.1;
      msg_->linear.y = 0.0;
      msg_->linear.z = 0.0;
      msg_->angular.x = 0.0;
      msg_->angular.y = 0.0;
      msg_->angular.z = 0.4;
      velocity_command_publisher_.publish(msg_);
    }
    //if we want to extend the safety state and we're within the time, just keep sending msg_
    else if (time_to_extend_bump_cliff_events_ > ros::Duration(1e-10) && 
	     ros::Time::now() - last_event_time_ < time_to_extend_bump_cliff_events_) {
      velocity_command_publisher_.publish(msg_);
    }
  }
};

} // namespace ca

#endif /* SAFETY_CONTROLLER_HPP_ */
