//
// Copyright (C) 2021 Jiwung Choi <jiwung@gmail.com>
//
// This file is part of ADP (Autonomous Driving Planner).
//

#include "adp/trajectory/path.h"
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>

namespace adp {

Path::id_type Path::unique_id_ = 1;

double ThreePointsCurvatureApproximation(const double x1, const double y1, const double x2, const double y2,
                                         const double x3, const double y3) {
  /// compute the area of triangle (p1, p2, p3) based on
  /// https://mathworld.wolfram.com/TriangleArea.html
  Eigen::Matrix3d A;
  A << x1, y1, 1, x2, y2, 1, x3, y3, 1;
  const double area = 0.5 * A.determinant();

  if (std::abs(area) < std::numeric_limits<double>::epsilon()) { // area == 0
    /// the three points p1, p2, p3 are collinear
    /// https://mathworld.wolfram.com/Collinear.html
    return 0;
  } else {
    /// https://www.kurims.kyoto-u.ac.jp/~kyodo/kokyuroku/contents/pdf/1111-16.pdf
    const double a = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    const double b = sqrt((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));
    const double c = sqrt((x1 - x3) * (x1 - x3) + (y1 - y3) * (y1 - y3));
    return (4 * area) / (a * b * c);
  }
}

double threePointsCurvatureApproximation(const std::vector<geometry_msgs::PoseStamped> &poses, const size_t i) {
  if (poses.size() >= 3 and 0 < i and i < poses.size() - 1) {
    return ThreePointsCurvatureApproximation(poses[i - 1].pose.position.x, poses[i - 1].pose.position.y,
                                             poses[i].pose.position.x, poses[i].pose.position.y,
                                             poses[i + 1].pose.position.x, poses[i + 1].pose.position.y);
  }
  return 0.;
}

double distanceInterval(const std::vector<geometry_msgs::PoseStamped> &poses, const size_t i) {
  if (i > 0) {
    const auto dx = poses[i].pose.position.x - poses[i - 1].pose.position.x;
    const auto dy = poses[i].pose.position.y - poses[i - 1].pose.position.y;
    return sqrt(dx * dx + dy * dy);
  }
  return std::numeric_limits<double>::max();
}

void Path::update(const nav_msgs::Path::ConstPtr &msg) {
  const size_t num_points = msg->poses.size();

  /// \note path_points_ and s_ are constructed by using push_back() which is less efficient than resize().
  /// This is because msg can contain two consecutive points that are almost identical, which can cause
  /// excessive heading difference. So, such point will be filtered out.
  path_points_.clear();
  s_.clear();

  // update x, y, theta, kappa, s
  for (size_t i = 0; i < num_points; i++) {
    const auto &pose = msg->poses[i].pose;
    const auto ds = distanceInterval(msg->poses, i);
    if (ds < resolution_) {
      continue;
    }
    PathPoint state;
    state.x = pose.position.x;
    state.y = pose.position.y;
    state.theta = tf::getYaw(pose.orientation);
    state.kappa = threePointsCurvatureApproximation(msg->poses, i);
    path_points_.push_back(state);
    const double s = (s_.empty() ? 0. : s_.back() + ds);
    s_.push_back(s);
  }

  // two end points curvatures are assigned same as adjacent point curvature.
  if (path_points_.size() >= 3) {
    path_points_[0].kappa = path_points_[1].kappa;
    path_points_[num_points - 1].kappa = path_points_[num_points - 2].kappa;
  }
}

void Path::update(const std::vector<double> &x, const std::vector<double> &y) {
  const size_t num_points = x.size();
  path_points_.resize(num_points);

  // update x, y, theta
  for (size_t i = 0; i < num_points; i++) {
    auto &point = path_points_[i];
    point.x = x[i];
    point.y = y[i];
    if (i + 1 < num_points) {
      const double dx = x[i + 1] - x[i];
      const double dy = y[i + 1] - y[i];
      point.theta = atan2(dy, dx);
    }
  }
  if (num_points > 1) {
    path_points_[num_points - 1].theta = path_points_[num_points - 2].theta;
  }

  // approximate curvature on eacy waypoint
  if (num_points >= 3) {
    // curvatures on intermediate points are determined by  ThreePointsCurvatureApproximation()
    for (size_t i = 1; i < num_points - 1; i++) {
      path_points_[i].kappa =
          ThreePointsCurvatureApproximation(path_points_[i - 1].x, path_points_[i - 1].y, path_points_[i].x,
                                            path_points_[i].y, path_points_[i + 1].x, path_points_[i + 1].y);
    }
    // two end points curvatures are assigned same as adjacent point curvature.
    path_points_[0].kappa = path_points_[1].kappa;
    path_points_[num_points - 1].kappa = path_points_[num_points - 2].kappa;
  }

  // update accumulated distances
  s_.resize(num_points, 0);
  for (size_t i = 1; i < num_points; i++) {
    const auto dx = path_points_[i].x - path_points_[i - 1].x;
    const auto dy = path_points_[i].y - path_points_[i - 1].y;
    s_[i] = s_[i - 1] + sqrt(dx * dx + dy * dy);
  }
}

int Path::getAssociatedIndex(const double s) const {
  if (s < 0.) {
    // extrapolate behind the first waypoint
    return 0;
  }
  const auto sf = s_.back();
  if (s > sf) {
    // extrapolate ahead the last waypoint
    return static_cast<int>(path_points_.size()) - 2;
  }
  for (size_t i = 0; i < s_.size() - 1; i++) {
    if (s_[i + 1] < s) {
      continue;
    }
    // s is within the interval [s_[i], s_[i + 1]]
    return static_cast<int>(i);
  }
  return -1;
}

PathPoint Path::getPathPoint(const double s) const {
  const int ind{getAssociatedIndex(s)}; // associated index to s
  if (ind < 0) {
    return {}; // return nan path state
  }
  size_t i = static_cast<size_t>(ind);
  return linearInterpolate(s, s_[i], s_[i + 1], path_points_[i], path_points_[i + 1]);
}

std::vector<double> Path::getVectorX() const {
  std::vector<double> x_vec;
  std::transform(path_points_.begin(), path_points_.end(), std::back_inserter(x_vec),
                 [](PathPoint const &p) { return p.x; });
  return x_vec;
}

std::vector<double> Path::getVectorY() const {
  std::vector<double> y_vec;
  std::transform(path_points_.begin(), path_points_.end(), std::back_inserter(y_vec),
                 [](PathPoint const &p) { return p.y; });
  return y_vec;
}

std::vector<double> Path::getVectorTheta() const {
  std::vector<double> theta_vec;
  std::transform(path_points_.begin(), path_points_.end(), std::back_inserter(theta_vec),
                 [](PathPoint const &p) { return p.theta; });
  return theta_vec;
}

std::vector<double> Path::getVectorKappa() const {
  std::vector<double> kappa_vec;
  std::transform(path_points_.begin(), path_points_.end(), std::back_inserter(kappa_vec),
                 [](PathPoint const &p) { return p.kappa; });
  return kappa_vec;
}

} // namespace adp
