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
    static RobotState& getInstance() {
        static RobotIdleState instance;
        return instance;
    }
 
    ERESULT startBuildMap();
    ERESULT startNavi(ENAVITYPE type);
private:
    RobotIdleState() {};
    virtual ~RobotIdleState() {};
    RobotIdleState(const RobotIdleState&) {};
    RobotIdleState& operator=(const RobotIdleState&) {};
};

} // end_ns

#endif
