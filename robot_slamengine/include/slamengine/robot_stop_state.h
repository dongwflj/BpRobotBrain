/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_STOP_STATE_H_
#define _ROBOT_STOP_STATE_H_

#include "robot_state.h"

namespace slamengine
{

class RobotStopState : public RobotState {
public:
    virtual ~RobotStopState() {}

    ERESULT startBuildMap();
};

} // end_ns

#endif
