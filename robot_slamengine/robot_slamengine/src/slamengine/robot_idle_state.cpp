/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/

#include <ros/ros.h>
#include "slamengine/robot_idle_state.h"
#include "slamengine/robot_motioning_state.h"
#include "slamengine/robot_mapping_state.h"
#include "slamengine/robot_fsm.h"
#include "slamengine/robot_ctrl.h"

namespace slamengine
{

ERESULT RobotIdleState::StartBuildMap(string task_id) {
    ROS_INFO("RobotIdleState::startBuildMap entry");
    // Impl build map operation
    context_->transitionTo(RobotMappingState::getInstance());    
    ROS_INFO("RobotIdleState::startBuildMap exit");
    return E_OK;
}

ERESULT RobotIdleState::StartNavi(string task_id, ENAVITYPE type, string goal_name, PixelPose goal_pose) {
    ROS_INFO("RobotIdleState::startNavi entry");
    context_->getRobotCtrl().StartNavi(task_id, type, goal_name, goal_pose);
    context_->transitionTo(RobotMotioningState::getInstance());    
    ROS_INFO("RobotIdleState::startNavi exit");
    return E_OK;
}

} // end_ns

