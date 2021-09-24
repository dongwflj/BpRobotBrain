/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/

#include <ros/ros.h>
#include "define.h"
#include "robot_engine_observer.h"
#include "robot_motioning_state.h"
#include "robot_mapping_state.h"
#include "robot_idle_state.h"
#include "robot_stop_state.h"
#include "robot_fsm.h"
#include "robot_ctrl.h"

namespace slamengine
{

ERESULT RobotMotioningState::NaviDoneEvent(const std::string& task_id, int32 state, const std::string& description) {
    ERESULT res = E_OK;
    ROS_INFO("RobotMotioningState::naviDoneEvent entry");
    const BlackBoard& bb = GetBlackBoard();
    context_->GetRobotEngineObserver().OnNaviDone(bb.task_id_, state, description);
    context_->TransitionTo(RobotIdleState::GetInstance());    
    ROS_INFO("RobotMotioningState::naviDoneEvent exit");
    return res; 
}

ERESULT RobotMotioningState::NotifyDropEvent(DropType type){
    ERESULT res = E_OK;
    if(type == CLIFF_DETECTED_AHEAD || type == CLIFF_DETECTED_BACK ) {
        const BlackBoard& bb = GetBlackBoard();
        res = context_->GetRobotCtrl().StopNavi(bb.task_id_);

        RobotState& state = RobotStopState::GetInstance();
        BlackBoard blackboard = bb;
        blackboard.from_state_ = this;
        state.SetBlackBoard(blackboard);
        context_->TransitionTo(state);    
        // Important, propagate event to target state
        context_->NotifyDropEvent(type);
    }
    return res; 
}

ERESULT RobotMotioningState::NotifyCollisionEvent(CollisionType type){
    ERESULT res = E_OK;
    if(type == COLLISION_DETECTED) {
        const BlackBoard& bb = GetBlackBoard();
        res = context_->GetRobotCtrl().StopNavi(bb.task_id_);
        RobotState& state = RobotStopState::GetInstance();
        BlackBoard blackboard = bb;
        blackboard.from_state_ = this;
        state.SetBlackBoard(blackboard);
        context_->TransitionTo(state);    
        context_->NotifyCollisionEvent(type);
    }
    return res; 
}

ERESULT RobotMotioningState::NotifyEStopEvent(EmergencyType type){
    ERESULT res = E_OK;
    if(EMERGENCY_DOWN == type) {
        const BlackBoard& bb = GetBlackBoard();
        res = context_->GetRobotCtrl().StopNavi(bb.task_id_);
        RobotState& state = RobotStopState::GetInstance();
        BlackBoard blackboard = bb;
        blackboard.from_state_ = this;
        state.SetBlackBoard(blackboard);
        context_->TransitionTo(state);    
        context_->NotifyEStopEvent(type);
    }
    return res; 
}

ERESULT RobotMotioningState::NotifyChargingEvent(const BatteryState& state){
    ERESULT res = E_OK;
    return res; 
}

ERESULT RobotMotioningState::NotifyCriticalHwErrEvent(){
    ERESULT res = E_OK;
    const BlackBoard& bb = GetBlackBoard();
    res = context_->GetRobotCtrl().StopNavi(bb.task_id_);

    RobotState& state = RobotStopState::GetInstance();
    BlackBoard blackboard = bb;
    blackboard.from_state_ = this;
    state.SetBlackBoard(blackboard);
    context_->TransitionTo(state);    
 
    context_->NotifyCriticalHwErrEvent();
    return res; 
}

ERESULT RobotMotioningState::Resume() {
    ROS_INFO("RobotMotioningState::Resume trigger, resume navigation");
    ERESULT res = E_OK;
    const BlackBoard& bb = GetBlackBoard();
    res = context_->GetRobotCtrl().StartNavi(bb.task_id_, bb.type_, bb.goal_name_, bb.goal_pose_);
    return res;
}

} // end_ns

