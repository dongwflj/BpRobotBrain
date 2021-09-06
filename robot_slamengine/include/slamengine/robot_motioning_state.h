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
    static RobotState& getInstance() {
        static RobotMotioningState instance;
        return instance;
    }
 
    ERESULT naviDoneEvent();
private:
    RobotMotioningState() {};
    virtual ~RobotMotioningState() {};
    RobotMotioningState(const RobotMotioningState&) {};
    RobotMotioningState& operator=(const RobotMotioningState&) {};
};

} // end_ns

#endif
