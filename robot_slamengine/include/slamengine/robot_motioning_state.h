/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_MOTIONING_STATE_H_
#define _ROBOT_MOTIONING_STATE_H_

#include "robot_state.h"

namespace slamengine
{

class RobotMotioningState : public RobotState {
public:
    virtual ~RobotMotioningState() {}

    ERESULT startBuildMap();
	ERESULT onNaviDone();
    ERESULT onNaviActive();
    ERESULT onNaviProgress();
};

} // end_ns

#endif
