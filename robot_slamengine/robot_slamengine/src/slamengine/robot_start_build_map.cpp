/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/
#include <stdexcept>
#include <ros/ros.h>
#include <string>

#include "slamengine/define.h"
#include "slamengine/robot_start_build_map.h"

namespace slamengine
{
BT::NodeStatus RobotStartBuildMap::tick() {
    ROS_INFO("RobotStartBuildMap entry");
    auto res = getInput<std::string>("para");  
    std::cout << "StartBuildMap: " << res.value() << std::endl; 
    config().blackboard->set("response", "this is build map resp");
    ROS_INFO("RobotStartBuildMap exit");
    return BT::NodeStatus::SUCCESS;
}

} // end_ns

