/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/

#include <ros/ros.h>
#include "robot_stop_state.h"
#include "robot_mapping_state.h"
#include "robot_motioning_state.h"
#include "robot_fsm.h"

namespace slamengine
{

ERESULT RobotStopState::NotifyDropEvent(DropType type){
    ERESULT res = E_OK;
    if(CLIFF_RESUMED_AHEAD == type || CLIFF_RESUMED_BACK == type) {
        //handle block event pop, todo
        ClearBlockEvent((int)type);
        //if no block, resume
        if(!block_){
            const BlackBoard& bb = GetBlackBoard();
            if (bb.from_state_ != nullptr) {
                context_->TransitionTo(*(bb.from_state_));
                context_->Resume();
            }
        }
    }
    else {
        ROS_INFO("RobotStopState::NotifyDropEvent repeated event");
    }
    return res; 
}

ERESULT RobotStopState::NotifyCollisionEvent(CollisionType type){
    ERESULT res = E_OK;
    if(COLLISION_RESUMED == type) {
        //handle block event pop, todo
        ClearBlockEvent((int)type);
        //if no block, resume
        if(!block_){
            const BlackBoard& bb = GetBlackBoard();
            if (bb.from_state_ != nullptr) {
                context_->TransitionTo(*(bb.from_state_));
                context_->Resume();
            }
        }
    }else{
        ROS_INFO("RobotStopState::NotifyCollisionEvent repeated event");
    }

    return res; 
}

ERESULT RobotStopState::NotifyEStopEvent(EmergencyType type){
    ERESULT res = E_OK;
    if(EMERGENCY_UP == type){
        //handle block event pop, todo
        ClearBlockEvent((int)type);
        //if no block, resume
        if(!block_){
            const BlackBoard& bb = GetBlackBoard();
            if (bb.from_state_ != nullptr) {
                context_->TransitionTo(*(bb.from_state_));
                context_->Resume();
            }
        }
    }else{
        ROS_INFO("RobotStopState::NotifyEStopEvent repeated event");
    }

    return res; 
}

void RobotStopState::SetBlockEvent(int block_event){
    std::set<int>::iterator it = block_event_set_.begin();
    while(it != block_event_set_.end()){
        if( *it == block_event ){
            ROS_INFO("RobotStopState::SetBLockEvent block event(%d) already exists", block_event);
            return;
        }
        ++it;
    }
  
    block_event_set_.insert(block_event);

    if(!block_){
        block_ = true;
        ROS_INFO("RobotStopState::SetBLockEvent set block_ flag");
    }

    ROS_INFO("RobotStopState::SetBLockEvent insert block event(%d)", block_event);
}

void RobotStopState::ClearBlockEvent(int block_event){
    ROS_INFO("RobotStopState::ClearBlockEvent block event(%d), block_event_set's size(%d), 1", block_event, (int)block_event_set_.size());
    std::set<int>::iterator it = block_event_set_.begin();
    while(it != block_event_set_.end()){
        ROS_INFO("RobotStopState::ClearBlockEvent event(%d)", *it);
        if( *it == (block_event-1) ){
            ROS_INFO("RobotStopState::ClearBlockEvent erase block event(%d)", block_event);
            block_event_set_.erase(block_event-1);

            ROS_INFO("RobotStopState::ClearBlockEvent block_event_set's size(%d), 2", (int)block_event_set_.size());
            if(0 == block_event_set_.size()){
                block_ = false;
                ROS_INFO("RobotStopState::SetBLockEvent clear block_ flag");
            }

            return;
        }
        ++it;
    }

    ROS_INFO("RobotStopState::ClearBlockEvent clear block event(%d), no event that could be erased", block_event-1);
}

} // end_ns

