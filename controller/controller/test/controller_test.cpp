//
// Copyright (C) 2021 Jiwung Choi <jiwung@gmail.com>
//
// This file is part of ADP (Autonomous Driving Planner).
//

#include "adp/controller/controller.h"

#include <gtest/gtest.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

using namespace adp;

class ControllerTest : public ::testing::Test {
protected:
  void recompute(const std::string &bag_file, const double t_ub_sec) {
    rosbag::Bag bag;
    bag.open(bag_file, rosbag::bagmode::Read);

    rosbag::View view(bag, rosbag::TopicQuery(required_topics_));
    for (const auto &m : view) {
      {
        const auto &msg_ptr = m.instantiate<nav_msgs::Odometry>();
        if (msg_ptr && msg_ptr->header.stamp.toSec() < t_ub_sec) {
          controller_.updateEgoOdometry(msg_ptr);
        }
      }
      {
        const auto &msg_ptr = m.instantiate<nav_msgs::Path>();
        if (msg_ptr && msg_ptr->header.stamp.toSec() < t_ub_sec) {
          controller_.updateWaypoints(msg_ptr);
        }
      }
      {
        const auto &msg_ptr = m.instantiate<visualization_msgs::Marker>();
        if (msg_ptr && msg_ptr->header.stamp.toSec() < t_ub_sec) {
          if (msg_ptr->ns.compare("pure_pursuit_target_point") == 0) {
            target_point_msg_ = msg_ptr;
          } else if (msg_ptr->ns.compare("pure_pursuit_nominal_path") == 0) {
            nominal_path_msg_ = msg_ptr;
          }
        }
      }
    }
    bag.close();

    controller_.step();
  }

  Controller controller_;
  const std::vector<std::string> required_topics_{"/carla/ego_vehicle/odometry", "/carla/ego_vehicle/waypoints",
                                                  "/adp/controller/pure_pursuit/target",
                                                  "/adp/controller/pure_pursuit/path"};
  visualization_msgs::Marker::ConstPtr target_point_msg_{nullptr};
  visualization_msgs::Marker::ConstPtr nominal_path_msg_{nullptr};
};

TEST_F(ControllerTest, TargetPoint) {
  std::string test_bag = ros::package::getPath("controller") + "/test/data/demo_lane_follow_with_const_velocity.bag";
  const double time_to_recompute{56}; // must be in [53, 60)
  recompute(test_bag, time_to_recompute);
  // get recomputed target and nominal path
  const auto &target_point = controller_.target_point();
  const auto &nominal_path = controller_.nominalPath();

  EXPECT_NEAR(target_point.x, target_point_msg_->pose.position.x, 0.5);
  EXPECT_NEAR(target_point.y, target_point_msg_->pose.position.y, 0.5);
  EXPECT_EQ(nominal_path.size(), nominal_path_msg_->points.size());
  for (auto i = 0u; i < nominal_path_msg_->points.size(); i++) {
    EXPECT_NEAR(nominal_path[i].x(), nominal_path_msg_->points[i].x, 0.5);
    EXPECT_NEAR(nominal_path[i].y(), nominal_path_msg_->points[i].y, 0.5);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
