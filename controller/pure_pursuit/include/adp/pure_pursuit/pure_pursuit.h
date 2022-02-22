//
// Copyright (C) 2021 Jiwung Choi <jiwung@gmail.com>
//
// This file is part of ADP (Autonomous Driving Planner).
//

#pragma once

#include "adp/reference_point_tracker/reference_point_tracker.h"
#include "adp/trajectory/path.h"

#include <eigen3/Eigen/Geometry>
#include <vector>
#include <visualization_msgs/Marker.h>

namespace adp {

///
/// \brief The PurePursuit controller class
/// "Implementation of the Pure Pursuit Path tracking Algorithm", R. Conlter, 1992
/// https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
///
class PurePursuit {
public:
  PurePursuit(const double lookahead_distance = 10.)
      : lookahead_distance_(lookahead_distance),
        target_point_tracker_(std::make_unique<TargetDistanceErrorEvaluator>(lookahead_distance)) {}

  void updateReferencePath(const std::shared_ptr<Path> &reference_path);
  void updateEgoVehiclePose(const std::shared_ptr<Eigen::Isometry3d> &map_to_ego_pose);

  ///
  /// \brief computes the curvature command based on the pure pursuit algorithm
  ///        https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
  /// \param map_to_ego  ego vehicle pose in map frame
  /// \param map_to_target  target point in map frame
  /// \return the requested curvature to follow the target point with circular motion.
  ///
  static double calculateCurvature(const Eigen::Isometry3d &map_to_ego, const Eigen::Vector3d &map_to_target);
  double calculateCurvature();

  //@{
  /** visualization */
  static std::vector<Eigen::Vector2d> nominalPath(const Eigen::Isometry2d &map_to_ego,
                                                  const Eigen::Vector2d &map_to_target, const double curvature);
  static std::vector<Eigen::Vector3d> nominalPath(const Eigen::Isometry3d &map_to_ego,
                                                  const Eigen::Vector3d &map_to_target, const double curvature);
  std::vector<Eigen::Vector3d> nominalPath() const;
  visualization_msgs::Marker nominalPathVisMsg() const;
  visualization_msgs::Marker targetPointVisMsg() const;
  //@}

  //@{
  /** accessors */
  double lookahead_distance() const { return lookahead_distance_; }
  void set_lookahead_distance(const double L) { lookahead_distance_ = L; }
  const PathPoint &target_point() const { return target_point_; }
  //@}

private:
  double lookahead_distance_{10.};
  double curvature_cmd_{0.};
  std::shared_ptr<const Path> reference_path_{nullptr};
  std::shared_ptr<const Eigen::Isometry3d> map_to_ego_pose_{nullptr};
  ReferencePointTracker target_point_tracker_;
  PathPoint target_point_;
};

} // namespace adp
