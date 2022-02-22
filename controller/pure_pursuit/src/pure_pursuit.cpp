//
// Copyright (C) 2021 Jiwung Choi <jiwung@gmail.com>
//
// This file is part of ADP (Autonomous Driving Planner).
//

#include "adp/pure_pursuit/pure_pursuit.h"
#include <cmath>
#include <eigen_conversions/eigen_msg.h>
#include <ros/console.h>

namespace adp {

void PurePursuit::updateReferencePath(const std::shared_ptr<Path> &reference_path) {
  target_point_tracker_.updateReferencePath(reference_path);
  reference_path_ = reference_path;
}

void PurePursuit::updateEgoVehiclePose(const std::shared_ptr<Eigen::Isometry3d> &map_to_ego_pose) {
  target_point_tracker_.updateAgentPose(map_to_ego_pose);
  map_to_ego_pose_ = map_to_ego_pose;
}

double PurePursuit::calculateCurvature(const Eigen::Isometry3d &map_to_ego, const Eigen::Vector3d &map_to_target) {
  // `map_to_target = map_to_ego_ * ego_to_target` is equivalent to
  const auto ego_to_target = map_to_ego.inverse() * map_to_target;
  const auto lookahead_dist = sqrt(ego_to_target.x() * ego_to_target.x() + ego_to_target.y() * ego_to_target.y());
  const double heading_to_target = atan2(ego_to_target.y(), ego_to_target.x());
  // https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
  return 2. * sin(heading_to_target) / lookahead_dist;
}

double PurePursuit::calculateCurvature() {
  if (reference_path_ == nullptr) {
    ROS_WARN("PurePursuit::calculateCurvature() - Can't calculate curvature because reference_path_ is uninitialized.");
    return 0.;
  }
  if (map_to_ego_pose_ == nullptr) {
    ROS_WARN(
        "PurePursuit::calculateCurvature() - Can't calculate curvature because map_to_ego_pose_ is uninitialized.");
    return 0.;
  }
  target_point_ = target_point_tracker_.findTargetPathPoint();
  {
    // check distance between ego and target point
    const auto dx = target_point_.x - map_to_ego_pose_->translation().x();
    const auto dy = target_point_.y - map_to_ego_pose_->translation().y();
    const auto l_error = sqrt(dx * dx + dy * dy) - lookahead_distance_;
    if (fabs(l_error) > 1.) {
      ROS_WARN("PurePursuit::calculateCurvature() - distance between map_to_ego_pose_ and target_point_ does not match "
               "to lookahead_distance_. (error: %f)",
               l_error);
    }
  }
  return curvature_cmd_ = calculateCurvature(*map_to_ego_pose_, target_point_.pos3D());
}

//@{
/** visualization */

// // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
double getYaw(const Eigen::Isometry3d &transform) {
  Eigen::Quaterniond q(transform.rotation());
  const double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
  return std::atan2(siny_cosp, cosy_cosp);
}

std::vector<Eigen::Vector2d> PurePursuit::nominalPath(const Eigen::Isometry2d &map_to_ego,
                                                      const Eigen::Vector2d &map_to_target, const double curvature) {
  std::vector<Eigen::Vector2d> map_to_path;
  map_to_path.push_back(map_to_ego.translation()); // ego pose as first point

  const auto dx = map_to_ego.translation().x() - map_to_target.x();
  const auto dy = map_to_ego.translation().y() - map_to_target.y();

  if (std::abs(curvature) > 1e-10 and dx * dx + dy * dy > 1.) {
    const auto ego_to_target = map_to_ego.inverse() * map_to_target;
    const double yaw_from_ego_to_target = atan2(ego_to_target.y(), ego_to_target.x());
    const double radius = 1. / curvature;
    const auto ego_to_center_circle = radius * Eigen::Vector2d::UnitY();
    const Eigen::Vector2d center_to_ego = -ego_to_center_circle;
    const Eigen::Vector2d center_to_target = center_to_ego + ego_to_target;
    auto temp = center_to_ego.dot(center_to_target) / (center_to_ego.norm() * center_to_target.norm());
    temp = std::min(1.0, std::max(-1.0, temp));
    auto center_angle = acos(temp);
    if (yaw_from_ego_to_target < 0) {
      center_angle = -center_angle;
    }

    static constexpr size_t n_path_segments{20};
    Eigen::Isometry2d t(Eigen::Isometry2d::Identity());
    t.translation() = ego_to_center_circle;
    for (size_t i = 1; i < n_path_segments; i++) {
      const double delta_yaw = center_angle * i / n_path_segments;
      t.matrix().block(0, 0, 2, 2) = Eigen::Rotation2Dd(delta_yaw).matrix();
      const auto ego_to_path_point = t * (center_to_ego);
      const auto map_to_path_point = map_to_ego * ego_to_path_point;
      map_to_path.push_back(map_to_path_point);
    }
  } // else: nearly zero curvature -> straight line connecting 'map_to_ego'
  // and 'map_to_target'
  map_to_path.push_back(map_to_target); // target pose as last point

  return map_to_path;
}

std::vector<Eigen::Vector3d> PurePursuit::nominalPath(const Eigen::Isometry3d &map_to_ego,
                                                      const Eigen::Vector3d &map_to_target, const double curvature) {
  const auto yaw = getYaw(map_to_ego);
  Eigen::Isometry2d map_to_ego_2d(Eigen::Isometry2d::Identity());
  map_to_ego_2d.translation() = map_to_ego.translation().head(2);
  map_to_ego_2d.matrix().block(0, 0, 2, 2) = Eigen::Rotation2Dd(yaw).matrix();
  const auto path_2d = nominalPath(map_to_ego_2d, Eigen::Vector2d(map_to_target.x(), map_to_target.y()), curvature);
  std::vector<Eigen::Vector3d> path_3d;
  const auto map_to_ego_z = map_to_ego.translation().z();
  for (const auto &p : path_2d) {
    path_3d.push_back({p.x(), p.y(), map_to_ego_z});
  }
  return path_3d;
}

std::vector<Eigen::Vector3d> PurePursuit::nominalPath() const {
  if (map_to_ego_pose_ == nullptr) {
    ROS_WARN_STREAM("PurePursuit::nominalPath() - map_to_ego_pose_ is uninitialized.");
    return {};
  }
  return nominalPath(*map_to_ego_pose_, target_point_.pos3D(), curvature_cmd_);
}

visualization_msgs::Marker PurePursuit::nominalPathVisMsg() const {
  if (map_to_ego_pose_ == nullptr) {
    ROS_WARN_STREAM(
        "PurePursuit::nominalPathVisMsg() - Can't generate message because map_to_ego_pose_ is uninitialized.");
    return {};
  }
  const std::vector<Eigen::Vector3d> nominal_path =
      nominalPath(*map_to_ego_pose_, target_point_.pos3D(), curvature_cmd_);

  // create LINE_STRIP type marker message to visualize nominal_path
  visualization_msgs::Marker path_marker;
  path_marker.header.frame_id = "map";
  path_marker.header.stamp = ros::Time::now();
  path_marker.ns = "pure_pursuit_nominal_path";
  path_marker.action = visualization_msgs::Marker::ADD;
  path_marker.type = visualization_msgs::Marker::LINE_STRIP;
  path_marker.pose.orientation.w = 1;
  path_marker.scale.x = 0.1;
  path_marker.color.b = 1.;
  path_marker.color.a = 1.;
  path_marker.points.resize(nominal_path.size());
  for (auto i = 0u; i < nominal_path.size(); i++) {
    tf::pointEigenToMsg(nominal_path[i], path_marker.points[i]);
  }
  return path_marker;
}

visualization_msgs::Marker PurePursuit::targetPointVisMsg() const {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "pure_pursuit_target_point";
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  tf::pointEigenToMsg(target_point_.pos3D(), marker.pose.position);
  marker.pose.orientation.w = 1;
  marker.color.g = 1.;
  marker.color.a = 1.;
  marker.scale.x = .7;
  marker.scale.y = .7;
  marker.scale.z = .7;
  return marker;
}

//@} // visualization

} // namespace adp
