/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_TELEOPING_STATE_H_
#define _ROBOT_TELEOPING_STATE_H_

#include "robot_state.h"

namespace slamengine
{

class RobotTeleOpingState : public RobotState {
public:
    virtual ~RobotTeleOpingState() {}

    ERESULT startBuildMap();
};

} // end_ns

#endif
