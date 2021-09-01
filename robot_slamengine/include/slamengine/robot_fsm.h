/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_FSM_H_
#define _ROBOT_FSM_H_

#include "define.h"

namespace slamengine
{
class RobotState;

class RobotFsm {
public:
    RobotFsm(RobotState *state);
    virtual ~RobotFsm();
    void TransitionTo(RobotState *state); 
// Event
    ERESULT startBuildMap();
    ERESULT stopBuildMap();
private:
    RobotState *state_;
};

} // end_ns

#endif
