/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/

#include <ros/ros.h>
#include "slamengine/robot_motioning_state.h"
#include "slamengine/robot_mapping_state.h"
#include "slamengine/robot_idle_state.h"
#include "slamengine/robot_fsm.h"
#include "slamengine/robot_ctrl.h"
#include "slamengine/robot_observer.h"

namespace slamengine
{

ERESULT RobotMotioningState::startBuildMap() {
    ROS_INFO("RobotMotioningState::startBuildMap entry");
    // Impl build map operation
    context_->transitionTo(new RobotMappingState());    
    ROS_INFO("RobotMotioningState::startBuildMap exit");
    return E_OK;
}

ERESULT RobotMotioningState::onNaviDone() {
    ROS_INFO("RobotMotioningState::onNaviDone entry");
	context_->getRobotObserver().onNaviDone();
    context_->transitionTo(new RobotIdleState());    
    ROS_INFO("RobotMotioningState::onNaviDone exit");
	return E_OK; 
}

ERESULT RobotMotioningState::onNaviActive() {
    ROS_INFO("RobotMotioningState::onNaviActive entry");
    ROS_INFO("RobotMotioningState::onNaviActive exit");
	return E_BADSTATE; 
}

ERESULT RobotMotioningState::onNaviProgress() {
    ROS_INFO("RobotMotioningState::onNaviProgress entry");
    ROS_INFO("RobotMotioningState::onNaviProgress exit");
	return E_BADSTATE; 
}

} // end_ns

