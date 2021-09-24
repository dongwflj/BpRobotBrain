/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_CHARGING_STATE_H_
#define _ROBOT_CHARGING_STATE_H_

#include <string>

#include "robot_state.h"

namespace slamengine
{

class RobotChargingState : public RobotState {
public:
    static RobotState& GetInstance() {
        static RobotChargingState instance;
        return instance;
    }
 
    ERESULT StartBuildMap(const std::string& task_id);
private:
    RobotChargingState() {};
    virtual ~RobotChargingState() {};
    RobotChargingState(const RobotChargingState&) {};
    RobotChargingState& operator=(const RobotChargingState&) {};
};

} // end_ns

#endif
