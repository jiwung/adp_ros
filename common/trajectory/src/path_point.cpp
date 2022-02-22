//
// Copyright (C) 2021 Jiwung Choi <jiwung@gmail.com>
//
// This file is part of ADP (Autonomous Driving Planner).
//

#include "adp/trajectory/path_point.h"
#include <tf/transform_datatypes.h>

namespace adp {

void PathPoint::update(const geometry_msgs::Pose &msg) {
  x = msg.position.x;
  y = msg.position.y;
  theta = tf::getYaw(msg.orientation);
}

std::ostream &operator<<(std::ostream &os, const PathPoint &path_point) {
  os << "(" << path_point.x << ", " << path_point.y << ")";
  return os;
}

Eigen::Vector3d PathPoint::pos3D() const { return {x, y, 0.}; }
Eigen::Vector2d PathPoint::forwardVector() const { return {cos(theta), sin(theta)}; }

Eigen::Isometry3d PathPoint::transform() const {
  Eigen::Isometry3d t{Eigen::Isometry3d::Identity()};
  t.translation().x() = x;
  t.translation().y() = y;
  t.rotate(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
  return t;
}

///
/// \brief linearInterpolate
/// \param s   distance value associated with the interpolated state
/// \param s1  distance associated with \p p1
/// \param s2  distance associated with \p p2
/// \param p1  state associated with \p s1
/// \param p2  state associated with \p s2
/// \return the path state p interpolating \p p1 and \p p2 such that
///         (s - s1) : (s2 - s1) = (p - p1) : (p2 - p1)
///
PathPoint linearInterpolate(const double s, const double s1, const double s2, const PathPoint &p1,
                            const PathPoint &p2) {
  PathPoint result;
  const auto t = (s - s1) / (s2 - s1);
  result.x = p1.x + t * (p2.x - p1.x);
  result.y = p1.y + t * (p2.y - p1.y);
  result.theta = p1.theta;
  result.kappa = p1.kappa + t * (p2.kappa - p1.kappa);
  return result;
}

} // namespace adp

namespace std {
bool isnan(const adp::PathPoint &p) { return isnan(p.x) or isnan(p.y) or isnan(p.theta) or isnan(p.kappa); }
} // namespace std
