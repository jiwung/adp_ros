//
// Copyright (C) 2021 Jiwung Choi <jiwung@gmail.com>
//
// This file is part of ADP (Autonomous Driving Planner).
//

#pragma once

#include "adp/trajectory/path_point.h"
#include <nav_msgs/Path.h>

namespace adp {

class Path {
public:
  using id_type = int64_t;

  Path(const double resolution = .1) : id_(unique_id_++), resolution_(resolution) {}

  void update(const nav_msgs::Path::ConstPtr &msg);
  void update(const std::vector<double> &x, const std::vector<double> &y);

  id_type id() const { return id_; }
  PathPoint operator[](const size_t i) const { return path_points_[i]; }
  PathPoint operator()(const double s) const { return getPathPoint(s); }
  PathPoint getPathPoint(const double s) const;
  size_t getPointsSize() const { return s_.size(); }
  int getAssociatedIndex(const double s) const;

  std::vector<double> getVectorX() const;
  std::vector<double> getVectorY() const;
  std::vector<double> getVectorTheta() const;
  std::vector<double> getVectorKappa() const;
  const std::vector<double> &getVectorS() const { return s_; }
  const std::vector<PathPoint> &getPathPoints() const { return path_points_; }
  double resolution() const { return resolution_; }

private:
  static id_type unique_id_;
  id_type id_{-1};
  std::vector<double> s_; //!< s_[i] is accumulated distance upto i-th discrete points
  std::vector<PathPoint> path_points_;
  const double resolution_{.1};
};

} // namespace adp
