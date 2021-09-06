/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_CHARGING_STATE_H_
#define _ROBOT_CHARGING_STATE_H_

#include "robot_state.h"

namespace slamengine
{

class RobotChargingState : public RobotState {
public:
    static RobotState& getInstance() {
        static RobotChargingState instance;
        return instance;
    }
 
    ERESULT startBuildMap();
private:
    RobotChargingState() {};
    virtual ~RobotChargingState() {};
    RobotChargingState(const RobotChargingState&) {};
    RobotChargingState& operator=(const RobotChargingState&) {};
};

} // end_ns

#endif
