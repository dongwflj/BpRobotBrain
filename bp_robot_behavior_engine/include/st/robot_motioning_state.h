/*
* Copyright (c) 2021, BytePowers, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_MOTIONING_STATE_H_
#define _ROBOT_MOTIONING_STATE_H_

#include <string>
#include "robot_state.h"

namespace slamengine
{

class RobotMotioningState : public RobotState {
public:
    static RobotState& GetInstance() {
        static RobotMotioningState instance;
        return instance;
    }
 
    ERESULT NaviDoneEvent(const std::string& task_id, int32 state, const std::string& description);
    ERESULT NotifyDropEvent(DropType type);
    ERESULT NotifyCollisionEvent(CollisionType type);
    ERESULT NotifyEStopEvent(EmergencyType type);
    ERESULT NotifyChargingEvent(const BatteryState& state);
    ERESULT NotifyCriticalHwErrEvent();
    ERESULT Resume(); 

private:
    RobotMotioningState() {};
    virtual ~RobotMotioningState() {};
    RobotMotioningState(const RobotMotioningState&) {};
    RobotMotioningState& operator=(const RobotMotioningState&) {};
};

} // end_ns

#endif
