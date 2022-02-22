//
// Copyright (C) 2021 Jiwung Choi <jiwung@gmail.com>
//
// This file is part of ADP (Autonomous Driving Planner).
//

#pragma once

#include "adp/controller/controller.h"

#include <ackermann_msgs/AckermannDrive.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

namespace adp {

class ControllerRos {
public:
  ControllerRos();
  void egoVehicleOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void waypointsCallBack(const nav_msgs::Path::ConstPtr &msg);
  void run();

private:
  ros::NodeHandle node_handle_;
  ros::Subscriber ego_odometry_subscriber_;
  ros::Subscriber waypoints_subscriber_;
  ros::Publisher ackermann_cmd_publisher_;
  ros::Publisher vis_nominal_path_publisher_;
  ros::Publisher vis_target_publisher_;
  ros::Publisher vis_ego_proj_publisher_;

  const double loop_rate_{20};

  Controller controller_;

  bool is_ego_odometry_initialized_{false};
  bool is_waypoints_initialized_{false};
};

} // namespace adp
