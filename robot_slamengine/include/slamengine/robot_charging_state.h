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
    virtual ~RobotChargingState() {}

    ERESULT startBuildMap();
};

} // end_ns

#endif
