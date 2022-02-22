//
// Copyright (C) 2021 Jiwung Choi <jiwung@gmail.com>
//
// This file is part of ADP (Autonomous Driving Planner).
//

#include "adp/controller/controller.h"
#include <cmath>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>

namespace adp {

void Controller::updateEgoOdometry(const nav_msgs::Odometry::ConstPtr &ego_odometry_msg) {
  if (map_to_ego_pose_ == nullptr) {
    map_to_ego_pose_.reset(new Eigen::Isometry3d);
    ego_projection_tracker_.updateAgentPose(map_to_ego_pose_);
    pure_pursuit_.updateEgoVehiclePose(map_to_ego_pose_);
  }
  tf::poseMsgToEigen(ego_odometry_msg->pose.pose, *map_to_ego_pose_);
}

void Controller::updateWaypoints(const nav_msgs::Path::ConstPtr &waypoints_msg) {
  if (reference_path_ == nullptr) {
    reference_path_.reset(new Path);
    ego_projection_tracker_.updateReferencePath(reference_path_);
    pure_pursuit_.updateReferencePath(reference_path_);
  }
  reference_path_->update(waypoints_msg);
}

bool Controller::step() {
  if (map_to_ego_pose_ == nullptr) {
    ROS_WARN("Controller::step() - map_to_ego_pose_ is uninitialized!");
    return false;
  }
  if (reference_path_ == nullptr) {
    ROS_WARN("Controller::step() - reference_path_ is uninitialized!");
    return false;
  }
  ego_projected_on_ref_path_ = ego_projection_tracker_.findTargetPathPoint();
  curvature_cmd_ = pure_pursuit_.calculateCurvature();
  steering_angle_cmd_ = atan(curvature_cmd_ * wheelbase_);
  return true;
}

double Controller::distanceToGoal() const {
  return reference_path_->getVectorS().back() - ego_projection_tracker_.getLatestTargetS();
}

// http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines
visualization_msgs::Marker Controller::egoProjectedFootVisMsg() const {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "ego_projected_foot_on_reference_path";
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;

  // linear interpolation between map_to_ego_pose_ and ego_projected_on_ref_path_
  auto &points = marker.points;
  static constexpr size_t n_points = 10;
  points.resize(n_points);
  points.front().x = map_to_ego_pose_->translation().x();
  points.front().y = map_to_ego_pose_->translation().y();
  points.front().z = map_to_ego_pose_->translation().z();
  points.back().x = ego_projected_on_ref_path_.x;
  points.back().y = ego_projected_on_ref_path_.y;
  points.back().z = map_to_ego_pose_->translation().z();
  for (auto i = 1u; i < n_points - 1; i++) {
    double t = static_cast<double>(i) / (n_points - 1);
    points[i].x = (1 - t) * points.front().x + t * points.back().x;
    points[i].y = (1 - t) * points.front().y + t * points.back().y;
    points[i].z = (1 - t) * points.front().z + t * points.back().z;
  }

  // linear interpolated transparent color
  auto &colors = marker.colors;
  colors.resize(n_points);
  for (auto i = 0u; i < n_points; i++) {
    double t = static_cast<double>(i) / (n_points - 1);
    colors[i].r = 1;
    colors[i].g = 1;
    colors[i].b = 1;
    colors[i].a = static_cast<float>((1 - t) + t * .1);
  }

  return marker;
}

} // namespace adp
