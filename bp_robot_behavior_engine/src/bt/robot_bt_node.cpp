/*
* Copyright (c) 2021, BytePowers, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/
#include <stdexcept>
#include <ros/ros.h>
#include <string>

#include "define.h"
#include "bt/robot_bt_node.h"

using namespace BT;
namespace bp
{

BT::NodeStatus Action_ReturnResponse::tick() {
    ROS_INFO("RobotReturnResponse entry");
    auto res = getInput<ERESULT>("inport_return_response");  
    std::cout << "Response: " << res.value() << std::endl; 
    config().blackboard->set<ERESULT>("robot_event_response", res.value());
    ROS_INFO("RobotReturnResponse exit");
    return BT::NodeStatus::FAILURE;
}

} // end_ns

