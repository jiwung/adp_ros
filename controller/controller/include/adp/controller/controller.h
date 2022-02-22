//
// Copyright (C) 2021 Jiwung Choi <jiwung@gmail.com>
//
// This file is part of ADP (Autonomous Driving Planner).
//

#pragma once

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include "adp/pure_pursuit/pure_pursuit.h"
#include "adp/reference_point_tracker/reference_point_tracker.h"
#include "adp/trajectory/path.h"

namespace adp {

class Controller {
public:
  Controller()
      : pure_pursuit_(10.0 /* lookahead_distance */),
        ego_projection_tracker_(std::make_unique<TargetProjectionErrorEvaluator>()), curvature_cmd_(0.),
        steering_angle_cmd_(0.) {}

  void updateEgoOdometry(const nav_msgs::Odometry::ConstPtr &ego_odometry_msg);
  void updateWaypoints(const nav_msgs::Path::ConstPtr &waypoints_msg);
  bool step();
  double distanceToGoal() const;
  bool isInitialized() const { return map_to_ego_pose_ != nullptr && reference_path_ != nullptr; }

  //@{
  /** accessors */
  const auto &map_to_ego_pose() const { return *map_to_ego_pose_; }
  const auto &reference_path() const { return *reference_path_; }
  double curvature_cmd() const { return curvature_cmd_; }
  double steering_angle_cmd() const { return steering_angle_cmd_; }
  const PathPoint &target_point() const { return pure_pursuit_.target_point(); }
  std::vector<Eigen::Vector3d> nominalPath() { return pure_pursuit_.nominalPath(); }
  //@}

  //@{
  /** visualization */
  visualization_msgs::Marker nominalPathVisMsg() const { return pure_pursuit_.nominalPathVisMsg(); }
  visualization_msgs::Marker targetPointVisMsg() const { return pure_pursuit_.targetPointVisMsg(); }
  visualization_msgs::Marker egoProjectedFootVisMsg() const;
  //@}

private:
  std::shared_ptr<Eigen::Isometry3d> map_to_ego_pose_{nullptr};
  std::shared_ptr<Path> reference_path_{nullptr};
  PathPoint ego_projected_on_ref_path_;
  PurePursuit pure_pursuit_;
  ReferencePointTracker ego_projection_tracker_;
  double curvature_cmd_{0.};
  double steering_angle_cmd_{0.};
  const double wheelbase_{2.87528}; // for Tesla Model3
};
} // namespace adp
