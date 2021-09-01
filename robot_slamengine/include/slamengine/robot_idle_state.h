/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_IDLE_STATE_H_
#define _ROBOT_IDLE_STATE_H_

#include "robot_state.h"

namespace slamengine
{

class RobotIdleState : public RobotState {
public:
    virtual ~RobotIdleState() {}

    ERESULT startBuildMap();
};

} // end_ns

#endif
