//
// Copyright (C) 2021 Jiwung Choi <jiwung@gmail.com>
//
// This file is part of ADPlanner (Autonomous Driving Planner).
//

#include "adp/reference_point_tracker/reference_point_tracker.h"
#include "ros/console.h"

namespace adp {

double TargetDistanceErrorEvaluator::evalLocalError(const Path &reference_path, const Eigen::Isometry3d &agent_pose,
                                                    const double s) const {
  const auto ref_point = reference_path.getPathPoint(s);
  const auto dx = ref_point.x - agent_pose.translation().x();
  const auto dy = ref_point.y - agent_pose.translation().y();
  const auto dist_between_agent_and_ref_point = sqrt(dx * dx + dy * dy);
  return dist_between_agent_and_ref_point - lookahead_distance_;
}

double TargetDistanceErrorEvaluator::evalGlobalError(const PathPoint &reference_point,
                                                     const Eigen::Isometry3d &agent_pose) const {
  const Eigen::Vector2d agent_poseition = agent_pose.translation().head(2);
  // forward vector (cos(yaw), sin(yaw)) of agent_pose, refer to
  // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  Eigen::Quaterniond q(agent_pose.rotation());
  Eigen::Vector2d agent_forward_vector{1 - 2 * (q.y() * q.y() + q.z() * q.z()), 2 * (q.w() * q.z() + q.x() * q.y())};
  const double weight_ratio{3.}; // cost weight ratio on angle_error vs distance_error
  const Eigen::Vector2d agent_to_ref_point{reference_point.x - agent_poseition.x(),
                                           reference_point.y - agent_poseition.y()};
  const auto d = agent_to_ref_point.norm();
  const auto distance_error = fabs(d - lookahead_distance_);
  // angle between agent_forward_vector and agent_to_ref_point
  const auto angle_error = acos(std::min(1., std::max(-1., agent_forward_vector.dot(agent_to_ref_point) / d)));
  return distance_error + weight_ratio * angle_error;
}

double TargetProjectionErrorEvaluator::evalLocalError(const Path &reference_path, const Eigen::Isometry3d &agent_pose,
                                                      const double s) const {
  const auto ref_point = reference_path.getPathPoint(s);
  // (ux, uy): vector from 'ref_point' to 'agent_pose'
  const double ux = agent_pose.translation().x() - ref_point.x;
  const double uy = agent_pose.translation().y() - ref_point.y;
  // (vx, vy): forward vector of 'ref_point'
  const double vx = cos(ref_point.theta);
  const double vy = sin(ref_point.theta);
  return ux * vx + uy * vy; // normality between (ux, uy) and (vx, vy)
}

double TargetProjectionErrorEvaluator::evalGlobalError(const PathPoint &reference_point,
                                                       const Eigen::Isometry3d &agent_pose) const {
  const auto dx = reference_point.x - agent_pose.translation().x();
  const auto dy = reference_point.y - agent_pose.translation().y();
  return dx * dx + dy * dy;
}

double ReferencePointTracker::findTargetS(const Path &reference_path, const Eigen::Isometry3d &agent_pose) {
  double s_initial_guess{latest_target_s_};
  const auto local_error = track_error_eval_->evalLocalError(reference_path, agent_pose, latest_target_s_);
  const auto global_error = track_error_eval_->evalGlobalError(reference_path(latest_target_s_), agent_pose);

  // if latest_target_s_ generates not a small error, finds the reference point leading to
  // the minimum global error and use it as initial guess for s
  if (fabs(local_error) > 5. || fabs(global_error) > 10.) {
    const auto min_error_ind = findMinErrorRefPointIndex(reference_path, agent_pose);
    s_initial_guess = reference_path.getVectorS()[min_error_ind];
  }

  // apply bisect method to find the solution s such that error_eval_f(s) = 0
  return latest_target_s_ = searchSForwardAndBackward(
             s_initial_guess, search_range_ds_, [&reference_path, &agent_pose, this](const double s) -> double {
               return track_error_eval_->evalLocalError(reference_path, agent_pose, s);
             });
}

PathPoint ReferencePointTracker::findTargetPathPoint(const Path &reference_path, const Eigen::Isometry3d &agent_pose) {
  return reference_path.getPathPoint(findTargetS(reference_path, agent_pose));
}

PathPoint ReferencePointTracker::findTargetPathPoint() {
  if (reference_path_ == nullptr) {
    ROS_WARN(
        "ReferencePointTracker::findTargetPathPoint() - Can't calculate because reference_path_ is uninitialized.");
    return {};
  }
  if (agent_pose_ == nullptr) {
    ROS_WARN("ReferencePointTracker::findTargetPathPoint() - Can't calculate because agent_pose_ is uninitialized.");
    return {};
  }
  return findTargetPathPoint(*reference_path_, *agent_pose_);
}

size_t ReferencePointTracker::findMinErrorRefPointIndex(const Path &reference_path,
                                                        const Eigen::Isometry3d &agent_pose) const {
  size_t min_error_ind{0};
  double min_error{std::numeric_limits<double>::max()};
  const auto n_points = reference_path.getPointsSize();
  for (auto i = 0u; i < n_points; i++) {
    const auto error = track_error_eval_->evalGlobalError(reference_path[i], agent_pose);
    if (error < min_error) {
      min_error = error;
      min_error_ind = i;
    }
  }
  return min_error_ind;
}

void ReferencePointTracker::updateReferencePath(const std::shared_ptr<const Path> &reference_path) {
  if (reference_path_ != nullptr) {
    if (reference_path_->id() != reference_path->id()) {
      latest_target_s_ = 0.;
    }
  }
  reference_path_ = reference_path;
}

void ReferencePointTracker::updateAgentPose(const std::shared_ptr<const Eigen::Isometry3d> &agent_pose) {
  agent_pose_ = agent_pose;
}

} // namespace adp
