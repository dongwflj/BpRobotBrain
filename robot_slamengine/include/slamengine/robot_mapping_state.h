/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_MAPPING_STATE_H_
#define _ROBOT_MAPPING_STATE_H_

#include "robot_state.h"

namespace slamengine
{

class RobotMappingState : public RobotState {
public:
    virtual ~RobotMappingState() {}

    ERESULT startBuildMap();
    ERESULT stopBuildMap();
};

} // end_ns

#endif
