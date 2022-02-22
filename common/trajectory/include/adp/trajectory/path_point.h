//
// Copyright (C) 2021 Jiwung Choi <jiwung@gmail.com>
//
// This file is part of ADP (Autonomous Driving Planner).
//

#pragma once

#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>

#define DOUBLE_NAN (std::numeric_limits<double>::quiet_NaN())

namespace adp {

struct PathPoint {
  void update(const geometry_msgs::Pose &msg);

  Eigen::Vector3d pos3D() const;
  Eigen::Vector2d forwardVector() const;
  Eigen::Isometry3d transform() const;

  double x{DOUBLE_NAN};     //!< cartesian x coordinate [m]
  double y{DOUBLE_NAN};     //!< cartesian y coordinate [m]
  double theta{DOUBLE_NAN}; //!< yaw angle [rad]
  double kappa{DOUBLE_NAN}; //!< curvature [1/m]
};

std::ostream &operator<<(std::ostream &os, const PathPoint &path_point);
PathPoint linearInterpolate(const double s, const double s1, const double s2, const PathPoint &p1, const PathPoint &p2);

} // namespace adp

namespace std {
bool isnan(const adp::PathPoint &p);
} // namespace std
