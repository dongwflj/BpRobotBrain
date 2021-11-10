/*
* Copyright (c) 2021, BytePowers, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_INIT_STATE_H_
#define _ROBOT_INIT_STATE_H_

#include <string>
#include "robot_state.h"

namespace slamengine
{

class RobotInitState : public RobotState {
public:
    static RobotState& GetInstance() {
        static RobotInitState instance;
        return instance;
    }
    ERESULT InitDoneEvent();
private:
    RobotInitState() {};
    virtual ~RobotInitState() {};
    RobotInitState(const RobotInitState&) {};
    RobotInitState& operator=(const RobotInitState&) {};
};

} // end_ns

#endif
