//
// Copyright (C) 2021 Jiwung Choi <jiwung@gmail.com>
//
// This file is part of ADP (Autonomous Driving Planner).
//

#include "adp/controller/controller_ros.h"

namespace adp {

ControllerRos::ControllerRos() {
  ego_odometry_subscriber_ =
      node_handle_.subscribe("/carla/ego_vehicle/odometry", 10, &ControllerRos::egoVehicleOdomCallback, this);
  waypoints_subscriber_ =
      node_handle_.subscribe("/carla/ego_vehicle/waypoints", 10, &ControllerRos::waypointsCallBack, this);
  ackermann_cmd_publisher_ =
      node_handle_.advertise<ackermann_msgs::AckermannDrive>("carla/ego_vehicle/ackermann_cmd", 10);
  vis_nominal_path_publisher_ =
      node_handle_.advertise<visualization_msgs::Marker>("adp/controller/pure_pursuit/path", 10);
  vis_target_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("adp/controller/pure_pursuit/target", 10);
  vis_ego_proj_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("adp/controller/ego_projection", 10);
}

void ControllerRos::egoVehicleOdomCallback(const nav_msgs::Odometry::ConstPtr &ego_odometry_msg) {
  controller_.updateEgoOdometry(ego_odometry_msg);
  is_ego_odometry_initialized_ = true;
}

void ControllerRos::waypointsCallBack(const nav_msgs::Path::ConstPtr &waypoints_msg) {
  controller_.updateWaypoints(waypoints_msg);
  is_waypoints_initialized_ = true;
}

void ControllerRos::run() {
  ros::Rate loop_rate(loop_rate_);
  while (ros::ok()) {
    ros::spinOnce();

    if (!is_ego_odometry_initialized_) {
      ROS_WARN("ControllerRos::run() - Ego vehicle odometry is not initialized yet ... ");
      loop_rate.sleep();
      continue;
    }

    if (!is_waypoints_initialized_) {
      ROS_ERROR("ControllerRos::run() - waypoints is not initialized yet ... ");
      loop_rate.sleep();
      continue;
    }

    controller_.step();

    float speed_cmd = 6.f;                    // let's drive with constant speed
    if (controller_.distanceToGoal() < 10.) { // close enough to goal?
      speed_cmd = 0.f;                        // then stop
    }

    // Ackermann control
    ackermann_msgs::AckermannDrive msg;
    msg.steering_angle = static_cast<float>(controller_.steering_angle_cmd());
    msg.steering_angle_velocity = 0;
    msg.speed = speed_cmd;
    msg.acceleration = 0;
    msg.jerk = 0;
    ackermann_cmd_publisher_.publish(msg);

    vis_nominal_path_publisher_.publish(controller_.nominalPathVisMsg());
    vis_target_publisher_.publish(controller_.targetPointVisMsg());
    vis_ego_proj_publisher_.publish(controller_.egoProjectedFootVisMsg());

    loop_rate.sleep();
  }
}

} // namespace adp
