/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_MOTIONPAUSE_STATE_H_
#define _ROBOT_MOTIONPAUSE_STATE_H_

#include "robot_state.h"

namespace slamengine
{

class RobotMotionPauseState : public RobotState {
public:
    virtual ~RobotMotionPauseState() {}

    ERESULT startBuildMap();
};

} // end_ns

#endif
