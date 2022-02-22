//
// Copyright (C) 2021 Jiwung Choi <jiwung@gmail.com>
//
// This file is part of ADPlanner (Autonomous Driving Planner).
//

#include "adp/reference_point_tracker/reference_point_tracker.h"

#include <gtest/gtest.h>

using namespace adp;

TEST(ReferencePointTrackerTest, ProjectedFootOnPath) {
  // map waypoints copied from Carla simulator
  std::vector<double> source_x{
      396.362, 396.364, 396.365, 396.366, 396.367, 396.368, 396.37,  396.371, 396.372, 396.373, 396.374, 396.376,
      396.377, 396.378, 396.38,  396.38,  396.38,  396.299, 396.083, 395.737, 395.264, 394.671, 393.963, 393.149,
      392.231, 391.224, 390.142, 388.999, 387.812, 386.595, 384.591, 384.591, 383.591, 382.591, 381.591, 380.591,
      379.591, 378.591, 377.591, 376.591, 375.591, 374.591, 373.591, 372.591, 371.591, 370.591, 369.591, 368.591,
      367.591, 366.591, 365.591, 364.591, 363.591, 362.591, 361.591, 360.591, 359.591, 358.591, 357.591, 356.591,
      355.591, 354.591, 353.591, 352.591, 351.591, 350.591, 349.591, 348.231, 348.231, 347.231, 346.231, 345.231,
      344.049, 342.824, 341.629, 340.484, 339.403, 338.403, 337.491, 336.69,  336.024, 335.504, 335.141, 334.938,
      334.895, 334.894, 334.893, 334.893, 334.893, 334.892};
  std::vector<double> source_y{
      -24.5424, -23.5424, -22.5424, -21.5424, -20.5424,  -19.5424, -18.5424, -17.5424,  -16.5424, -15.5424,
      -14.5424, -13.5424, -12.5424, -11.5424, -9.85205,  -9.85205, -8.826,   -7.60871,  -6.4078,  -5.23789,
      -4.1132,  -3.04741, -2.05347, -1.14241, -0.323511, 0.383901, 0.970377, 1.42808,   1.75089,  1.9345,
      1.98,     1.98,     1.98053,  1.98106,  1.98159,   1.98212,  1.98266,  1.98319,   1.98372,  1.98425,
      1.98478,  1.98531,  1.98584,  1.98637,  1.98691,   1.98744,  1.98797,  1.9885,    1.98903,  1.98956,
      1.99009,  1.99062,  1.99116,  1.99169,  1.99222,   1.99275,  1.99328,  1.99381,   1.99434,  1.99487,
      1.99541,  1.99594,  1.99647,  1.997,    1.99753,   1.99806,  1.99859,  1.99932,   1.99932,  1.99985,
      2.00038,  2.00091,  1.95818,  1.77145,  1.4392,    0.966207, 0.359304, -0.372764, -1.2287,  -2.20446,
      -3.2769,  -4.42747, -5.63624, -6.88232, -8.06962,  -9.06962, -10.7888, -10.7888,  -11.7888, -12.7888};

  Path path;
  path.update(source_x, source_y);

  ReferencePointTracker ego_projection_tracker(std::make_unique<TargetProjectionErrorEvaluator>());

  const int ind{15};
  const auto &path_point = path[ind];
  Eigen::Isometry3d ego_pos = path_point.transform();
  const double x_disturb{-2.}, y_disturb{-1.}, theta_disturb{.1};
  ego_pos.translation().x() += x_disturb;
  ego_pos.translation().y() += y_disturb;
  ego_pos.rotate(Eigen::AngleAxisd(theta_disturb, Eigen::Vector3d::UnitZ()));

  const auto min_error_ind = ego_projection_tracker.findMinErrorRefPointIndex(path, ego_pos);
  EXPECT_LE(std::abs(static_cast<int>(min_error_ind) - ind), 3);

  const auto ego_projected_foot = ego_projection_tracker.findTargetPathPoint(path, ego_pos);
  EXPECT_FALSE(std::isnan(ego_projected_foot));

  Eigen::Vector2d u(ego_pos.translation().x() - ego_projected_foot.x, ego_pos.translation().y() - ego_projected_foot.y);
  EXPECT_LE(u.norm(), 3);
  const Eigen::Vector2d v = ego_projected_foot.forwardVector(); // forward vector
  const auto dotp = u.dot(v);
  EXPECT_LE(fabs(dotp), 1e-6);
}

TEST(ReferencePointTrackerTest, LookaheadDistantTargetPoint) {
  // map waypoints copied from Carla simulator
  std::vector<double> source_x{1.95512, 1.9555,  1.95589, 1.95627, 1.95666, 1.95704, 1.95743, 1.95781,
                               1.9582,  1.95858, 1.95897, 1.95935, 1.95974, 1.96012, 1.96051, 1.96089,
                               1.96128, 1.96166, 1.96205, 1.96243, 1.96282, 1.9632,  1.96359, 1.96398,
                               1.96436, 1.96475, 1.96513, 1.96552, 1.9659,  1.96629};
  std::vector<double> source_y{-48.6008, -47.6008, -46.6008, -45.6008, -44.6008, -43.6008, -42.6008, -41.6008,
                               -40.6008, -39.6008, -38.6008, -37.6008, -36.6008, -35.6008, -34.6008, -33.6008,
                               -32.6008, -31.6008, -30.6008, -29.6008, -28.6008, -27.6008, -26.6008, -25.6008,
                               -24.6008, -23.6008, -22.6008, -21.6008, -20.6008, -19.6008};
  Path path;
  path.update(source_x, source_y);
  PathPoint ego_point;
  ego_point.x = 2.78904;
  ego_point.y = -41.6306;
  ego_point.theta = 1.50745;
  ego_point.kappa = 0;
  const Eigen::Isometry3d ego_pos = ego_point.transform();

  const double lookahead_distance{10.};
  ReferencePointTracker lookahead_tracker(std::make_unique<TargetDistanceErrorEvaluator>(lookahead_distance));

  const auto s_vec = path.getVectorS();
  //  s_initial_guess = s_vec[ind] + lookahead_distance;
  const auto target_point = lookahead_tracker.findTargetPathPoint(path, ego_pos); // (1.96164, -31.6648)
  const Eigen::Vector2d ego_to_target_point{target_point.x - ego_pos.translation().x(),
                                            target_point.y - ego_pos.translation().y()};
  EXPECT_NEAR(ego_to_target_point.norm(), lookahead_distance, 1e-6);

  // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  Eigen::Quaterniond q(ego_pos.rotation());
  const Eigen::Vector2d ego_forward_vector{
      1 - 2 * (q.y() * q.y() + q.z() * q.z()),
      2 * (q.w() * q.z() + q.x() * q.y()),
  };
  const auto angle_diff =
      acos(std::min(1., std::max(-1., ego_forward_vector.dot(ego_to_target_point) / ego_to_target_point.norm())));
  EXPECT_LE(angle_diff, M_PI / 4.);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
