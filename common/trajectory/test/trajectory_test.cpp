//
// Copyright (C) 2021 Jiwung Choi <jiwung@gmail.com>
//
// This file is part of ADP (Autonomous Driving Planner).
//

#include <gtest/gtest.h>

#include "adp/trajectory/path.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

using namespace adp;

template <class MESSAGE>
boost::shared_ptr<const MESSAGE> loadRosMsgConstPtr(const std::string &bag_file, const std::string &topic) {
  rosbag::Bag bag;
  bag.open(bag_file, rosbag::bagmode::Read);
  const std::vector<std::string> required_topics{topic};
  rosbag::View view(bag, rosbag::TopicQuery(required_topics));
  bool found{false};
  for (const auto &m : view) {
    const auto &msg_ptr = m.instantiate<MESSAGE>();
    if (msg_ptr) {
      return msg_ptr;
      found = true;
    }
  }
  bag.close();
  return nullptr;
}

TEST(PathTest, SanityCheck) {
  // read test bag file
  std::string test_bag = ros::package::getPath("trajectory") + "/test/data/carla_waypoints_sample.bag";
  const auto msg = loadRosMsgConstPtr<nav_msgs::Path>(test_bag, "/carla/ego_vehicle/waypoints");
  Path path;
  path.update(msg);
  const auto resolution = path.resolution();
  const auto &path_points = path.getPathPoints();
  const auto num_points = path_points.size();
  const auto &vec_s = path.getVectorS();
  for (auto i = 0u; i < num_points; i++) {
    EXPECT_FALSE(std::isnan(path_points[i]));
    if (i > 0) {
      const auto dx = path_points[i].x - path_points[i - 1].x;
      const auto dy = path_points[i].y - path_points[i - 1].y;
      const auto ds = vec_s[i] - vec_s[i - 1];
      EXPECT_NEAR(sqrt(dx * dx + dy * dy), ds, 1e-6);
      // check monotonicity of s
      EXPECT_GT(ds, resolution);
    }
  }

  PathPoint path_point = path(0.);
  EXPECT_FALSE(std::isnan(path_point));
  EXPECT_DOUBLE_EQ(path_point.x, path[0].x);
  EXPECT_DOUBLE_EQ(path_point.y, path[0].y);
  EXPECT_DOUBLE_EQ(path_point.theta, path[0].theta);
  EXPECT_DOUBLE_EQ(path_point.kappa, path[0].kappa);

  // linear interpolation
  size_t i = 0;  // first index
  double t = .3; // linear interpolation ratio
  const auto s1 = vec_s[i];
  const auto p1 = path[i];
  const auto s2 = vec_s[i + 1];
  const auto p2 = path[i + 1];
  const auto s = s1 + t * (s2 - s1);
  path_point = path(s);
  EXPECT_DOUBLE_EQ(path_point.x, p1.x + t * (p2.x - p1.x));
  EXPECT_DOUBLE_EQ(path_point.y, p1.y + t * (p2.y - p1.y));
  EXPECT_DOUBLE_EQ(path_point.theta, p1.theta);
  EXPECT_DOUBLE_EQ(path_point.kappa, p1.kappa + t * (p2.kappa - p1.kappa));
}

TEST(PathPointTest, conversion) {
  PathPoint path_point;
  path_point.x = 1;
  path_point.y = 2;
  path_point.theta = M_PI / 2.;
  path_point.kappa = 0;

  const auto pos_3d = path_point.pos3D();
  EXPECT_DOUBLE_EQ(pos_3d.x(), path_point.x);
  EXPECT_DOUBLE_EQ(pos_3d.y(), path_point.y);

  const auto forward_vector = path_point.forwardVector();
  EXPECT_DOUBLE_EQ(forward_vector.x(), cos(path_point.theta));
  EXPECT_DOUBLE_EQ(forward_vector.y(), sin(path_point.theta));

  const auto transform = path_point.transform();
  EXPECT_DOUBLE_EQ(transform.translation().x(), path_point.x);
  EXPECT_DOUBLE_EQ(transform.translation().y(), path_point.y);
  // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  Eigen::Quaterniond q(transform.rotation());
  const auto theta = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
  EXPECT_DOUBLE_EQ(theta, path_point.theta);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
