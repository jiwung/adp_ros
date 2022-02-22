//
// Copyright (C) 2021 Jiwung Choi <jiwung@gmail.com>
//
// This file is part of ADP (Autonomous Driving Planner).
//

#include <gtest/gtest.h>

#include "adp/pure_pursuit/pure_pursuit.h"

using namespace adp;

TEST(TestPurePursuit, PurePursuitCurvature) {
  // Suppose a circular reference path whose center is (0,0) with 'radius'
  const double radius{100.};
  // polar coordinate 'phi' determines the target point
  const double phi{M_PI / 3.};

  // Let ego vehicle's position is (radius,0) with heading of 90 deg
  Eigen::Isometry3d map_to_ego{Eigen::Isometry3d::Identity()};
  map_to_ego.translation().x() = radius;
  map_to_ego.translation().y() = 0;
  map_to_ego.rotate(Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitZ()));

  // Let target point is on the circle and determined by polar coordinate 'phi'
  Eigen::Vector3d map_to_target;
  map_to_target.x() = radius * cos(phi);
  map_to_target.y() = radius * sin(phi);
  map_to_target.z() = 0;

  // 1. Test curvature command
  const auto curvature_cmd = PurePursuit::calculateCurvature(map_to_ego, map_to_target);
  EXPECT_DOUBLE_EQ(curvature_cmd, 1. / radius);

  // 2. Test nominal path
  const auto nominal_path = PurePursuit::nominalPath(map_to_ego, map_to_target, curvature_cmd);
  // first point must be equal to map_to_ego
  EXPECT_DOUBLE_EQ(nominal_path.front().x(), map_to_ego.translation().x());
  EXPECT_DOUBLE_EQ(nominal_path.front().y(), map_to_ego.translation().y());
  // note that nominal_path must be a circular arc
  double accumul_phi{0.}; // accumulated polar coordinate to i-th nominal_path point
  for (auto i = 1u; i + 1 < nominal_path.size(); i++) {
    const auto v = nominal_path[i] - nominal_path[i - 1];
    const auto d = sqrt(v.x() * v.x() + v.y() * v.y()); // distance between (i-1)th and ith points
    // center angle of arc connecting (i-1)th and ith points
    const auto center_angle = 2 * asin((d / 2.) / radius);
    accumul_phi += center_angle;
    EXPECT_DOUBLE_EQ(nominal_path[i].x(), radius * cos(accumul_phi));
    EXPECT_DOUBLE_EQ(nominal_path[i].y(), radius * sin(accumul_phi));
  }
  // last point must be equal to map_to_target
  EXPECT_DOUBLE_EQ(nominal_path.back().x(), map_to_target.x());
  EXPECT_DOUBLE_EQ(nominal_path.back().y(), map_to_target.y());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
