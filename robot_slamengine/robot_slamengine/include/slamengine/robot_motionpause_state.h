/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_MOTIONPAUSE_STATE_H_
#define _ROBOT_MOTIONPAUSE_STATE_H_

#include "robot_state.h"
#include <string>
using namespace std;

namespace slamengine
{

class RobotMotionPauseState : public RobotState {
public:
    static RobotState& getInstance() {
        static RobotMotionPauseState instance;
        return instance;
    }
 
    ERESULT StartBuildMap(string task_id);
private:
    RobotMotionPauseState() {};
    virtual ~RobotMotionPauseState() {};
    RobotMotionPauseState(const RobotMotionPauseState&) {};
    RobotMotionPauseState& operator=(const RobotMotionPauseState&) {};
};

} // end_ns

#endif
