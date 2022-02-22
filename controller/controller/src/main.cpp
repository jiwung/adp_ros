//
// Copyright (C) 2021 Jiwung Choi <jiwung@gmail.com>
//
// This file is part of ADP (Autonomous Driving Planner).
//

#include <ros/ros.h>

#include "adp/controller/controller_ros.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "adp_controller");
  ros::Time::init();

  adp::ControllerRos controller_ros;
  controller_ros.run();

  return 0;
}
