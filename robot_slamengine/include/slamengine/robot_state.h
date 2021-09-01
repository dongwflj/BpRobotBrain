/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_STATE_H_
#define _ROBOT_STATE_H_

#include "robot_fsm.h"

namespace slamengine
{

class RobotState {
public:
    virtual ~RobotState() {}
    void set_context(RobotFsm *context) {
        this->context_ = context;
    }

    virtual ERESULT startBuildMap();
    virtual ERESULT stopBuildMap();
protected:
    RobotFsm *context_;
};

} // end_ns

#endif
