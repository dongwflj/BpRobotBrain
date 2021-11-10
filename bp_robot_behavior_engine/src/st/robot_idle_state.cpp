/*
* Copyright (c) 2021, BytePowers, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/

#include <ros/ros.h>
#include "robot_idle_state.h"
#include "robot_motioning_state.h"
#include "robot_mapping_state.h"
#include "robot_fsm.h"
#include "robot_ctrl.h"

namespace slamengine
{

ERESULT RobotIdleState::StartBuildMap(const std::string& task_id) {
    ROS_INFO("RobotIdleState::startBuildMap entry");
    ERESULT res = E_OK;
    res = context_->GetRobotCtrl().StartBuildMap(task_id);

    RobotState* state = &RobotMappingState::GetInstance();
    BlackBoard blackboard;
    blackboard.task_id_ = task_id;
    state->SetBlackBoard(blackboard);
    context_->TransitionTo(*state);    
    ROS_INFO("RobotIdleState::startBuildMap exit");
    return res;
}

ERESULT RobotIdleState::StartNavi(const std::string& task_id, ENAVITYPE type, const std::string& goal_name, const PixelPose& goal_pose) {
    ROS_INFO("RobotIdleState::startNavi entry");
    ERESULT res = E_OK;

    RobotState *state = &RobotMotioningState::GetInstance();

    res = context_->GetRobotCtrl().StartNavi(task_id, type, goal_name, goal_pose);
    if (res == E_OK) {
        BlackBoard blackboard;
        blackboard.task_id_ = task_id;
        blackboard.type_ = type;
        blackboard.goal_name_ = goal_name;
        blackboard.goal_pose_ = goal_pose;
        state->SetBlackBoard(blackboard);
        context_->TransitionTo(*state);    
    }
    else {
        ROS_WARN("RobotIdleState::startNavi return fail:%d", (int)res);
    }

    ROS_INFO("RobotIdleState::startNavi exit");
    return res;
}

ERESULT RobotIdleState::SaveMap(const std::string& task_id, const std::string& map_name) {
    ROS_INFO("RobotIdleState::SaveMap entry");
    ERESULT res = E_OK;
    res = context_->GetRobotCtrl().SaveMap(task_id, map_name);
    ROS_INFO("RobotIdleState::SaveMap exit");
    return res;
}

ERESULT RobotIdleState::LoadMap(const std::string& task_id, const std::string& map_name) { 
    ROS_INFO("RobotIdleState::LoadMap entry");
    ERESULT res = E_OK;
    res = context_->GetRobotCtrl().LoadMap(task_id, map_name);
    ROS_INFO("RobotIdleState::LoadMap exit");
    return res;
}

} // end_ns

