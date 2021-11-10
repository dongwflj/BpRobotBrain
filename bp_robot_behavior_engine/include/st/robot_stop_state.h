/*
* Copyright (c) 2021, BytePowers, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_STOP_STATE_H_
#define _ROBOT_STOP_STATE_H_

#include <string>
#include <set>
#include "robot_state.h"

namespace slamengine
{

class RobotStopState : public RobotState {
public:
    static RobotState& GetInstance() {
        static RobotStopState instance;
        return instance;
    }
 
    ERESULT NotifyDropEvent(DropType type);
    ERESULT NotifyCollisionEvent(CollisionType type);
    ERESULT NotifyEStopEvent(EmergencyType type);
    void SetBlockEvent(int block_event);
    void ClearBlockEvent(int block_event);
private:
    RobotStopState():block_(false) {};
    virtual ~RobotStopState() {};
    RobotStopState(const RobotStopState&) {};
    RobotStopState& operator=(const RobotStopState&) {};
    bool IsBlocked(){ return block_; }
private:
    bool block_;
    std::set<int> block_event_set_;
};

} // end_ns

#endif
