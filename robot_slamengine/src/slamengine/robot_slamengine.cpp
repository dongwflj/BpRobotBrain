/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/

#include <ros/ros.h>

#include "slamengine/robot_slamengine.h"

namespace slamengine
{

RobotSlamEngine::RobotSlamEngine() {
    ROS_INFO("NaviModule initializing...");
    ROS_INFO("NaviModule Initialization Done");
}

} // end_ns

