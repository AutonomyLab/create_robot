/**
 * @file /src/ca_bumper2pc.cpp
 *
 * @brief Bumper to pointcloud nodelet class implementation.
 *
 * 
 *
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <pluginlib/class_list_macros.h>

#include "ca_bumper2pc/ca_bumper2pc.hpp"

namespace create_bumper2pc
{

void Bumper2PcNodelet::bumperSensorCB(const ca_msgs::Bumper::ConstPtr& msg_bump)
{
  if (pointcloud_pub_.getNumSubscribers() == 0)
    return;

  // We publish just one "no events" pc (with all three points far away) and stop spamming when bumper/cliff conditions disappear
  if (! (msg_bump->is_left_pressed || msg_bump->is_right_pressed) && ! prev_bumper)
    return;

  prev_bumper = msg_bump->is_left_pressed || msg_bump->is_right_pressed;

  // For any of {left/center/right} with no bumper/cliff event, we publish a faraway point that won't get used 
  if (msg_bump->is_left_pressed && !(msg_bump->is_right_pressed))
  {
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[0].offset], &p_side_x_, sizeof(float));
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[1].offset], &p_side_y_, sizeof(float));
  }
  else
  {
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[1].offset], &P_INF_Y, sizeof(float));
  }

  if (msg_bump->is_left_pressed && msg_bump->is_right_pressed)
  {
    memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[0].offset], &pc_radius_, sizeof(float));
  }
  else
  {
    memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
  }

  if (msg_bump->is_right_pressed && !(msg_bump->is_left_pressed))     
  {
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[0].offset], &p_side_x_, sizeof(float));
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[1].offset], &n_side_y_, sizeof(float));
  }
  else
  {
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[1].offset], &N_INF_Y, sizeof(float));
  }

  pointcloud_.header.stamp = msg_bump->header.stamp;
  pointcloud_pub_.publish(pointcloud_);
}


void Bumper2PcNodelet::cliffSensorCB(const ca_msgs::Cliff::ConstPtr& msg_cliff)
{
  if (pointcloud_pub_.getNumSubscribers() == 0)
    return;

  // We publish just one "no events" pc (with all three points far away) and stop spamming when bumper/cliff conditions disappear
  if ( !(msg_cliff->is_cliff_left || msg_cliff->is_cliff_front_left || msg_cliff->is_cliff_front_right || msg_cliff->is_cliff_right)
	 && !prev_cliff)
    return;

  prev_cliff  = msg_cliff->is_cliff_left || msg_cliff->is_cliff_front_left || msg_cliff->is_cliff_front_right || msg_cliff->is_cliff_right;

  
  // For any of {left/center/right} with no bumper/cliff event, we publish a faraway point that won't get used 
  if (msg_cliff->is_cliff_left)
  {
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[0].offset], &p_side_x_, sizeof(float));
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[1].offset], &p_side_y_, sizeof(float));
  }
  else
  {
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[1].offset], &P_INF_Y, sizeof(float));
  }

  if (msg_cliff->is_cliff_front_left && msg_cliff->is_cliff_front_right)
  {
    memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[0].offset], &pc_radius_, sizeof(float));
  }
  else
  {
    memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
  }

  if (msg_cliff->is_cliff_right)
  {
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[0].offset], &p_side_x_, sizeof(float));
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[1].offset], &n_side_y_, sizeof(float));
  }
  else
  {
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[1].offset], &N_INF_Y, sizeof(float));
  }

  pointcloud_.header.stamp = msg_cliff->header.stamp;
  pointcloud_pub_.publish(pointcloud_);
}

void Bumper2PcNodelet::onInit()
{
  ros::NodeHandle nh = this->getPrivateNodeHandle();

  // Bumper/cliff pointcloud distance to base frame; should be something like the robot radius plus
  // costmap resolution plus an extra to cope with robot inertia. This is a bit tricky parameter: if
  // it's too low, costmap will ignore this pointcloud (the robot footprint runs over the hit obstacle),
  // but if it's too big, hit obstacles will be mapped too far from the robot and the navigation around
  // them will probably fail.
  std::string base_link_frame;
  double r, h, angle;
  nh.param("pointcloud_radius", r, 0.25); pc_radius_ = r;
  nh.param("pointcloud_height", h, 0.04); pc_height_ = h;
  nh.param("side_point_angle", angle, 0.34906585); 
  nh.param<std::string>("base_link_frame", base_link_frame, "/base_link");

  // Lateral points x/y coordinates; we need to store float values to memcopy later
  p_side_x_ = + pc_radius_*sin(angle); // angle degrees from vertical
  p_side_y_ = + pc_radius_*cos(angle); // angle degrees from vertical
  n_side_y_ = - pc_radius_*cos(angle); // angle degrees from vertical

  // Prepare constant parts of the pointcloud message to be  published
  pointcloud_.header.frame_id = base_link_frame;
  pointcloud_.width  = 3;
  pointcloud_.height = 1;
  pointcloud_.fields.resize(3);

  // Set x/y/z as the only fields
  pointcloud_.fields[0].name = "x";
  pointcloud_.fields[1].name = "y";
  pointcloud_.fields[2].name = "z";

  int offset = 0;
  // All offsets are *4, as all field data types are float32
  for (size_t d = 0; d < pointcloud_.fields.size(); ++d, offset += 4)
  {
    pointcloud_.fields[d].count    = 1;
    pointcloud_.fields[d].offset   = offset;
    pointcloud_.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
  }

  pointcloud_.point_step = offset;
  pointcloud_.row_step   = pointcloud_.point_step * pointcloud_.width;

  pointcloud_.data.resize(3 * pointcloud_.point_step);
  pointcloud_.is_bigendian = false;
  pointcloud_.is_dense     = true;

  // Bumper/cliff "points" fix coordinates (the others depend on sensor activation/deactivation)

  // y: always 0 for central bumper
  memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[1].offset], &ZERO, sizeof(float));

  // z: constant elevation from base frame
  memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height_, sizeof(float));
  memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height_, sizeof(float));
  memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height_, sizeof(float));

  pointcloud_pub_  = nh.advertise <sensor_msgs::PointCloud2> ("pointcloud", 10);
  cliff_event_subscriber_ = nh.subscribe("cliff", 10, &Bumper2PcNodelet::cliffSensorCB, this);
  bumper_event_subscriber_ = nh.subscribe("bumper", 10, &Bumper2PcNodelet::bumperSensorCB, this);

  ROS_INFO("Bumper/cliff pointcloud configured at distance %f and height %f from base frame", pc_radius_, pc_height_);
}

} // namespace create_bumper2pc


PLUGINLIB_EXPORT_CLASS(create_bumper2pc::Bumper2PcNodelet, nodelet::Nodelet);
