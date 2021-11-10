/*
* Copyright (c) 2021, BytePowers, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_MOTIONPAUSE_STATE_H_
#define _ROBOT_MOTIONPAUSE_STATE_H_

#include <string>
#include "robot_state.h"

namespace slamengine
{

class RobotMotionPauseState : public RobotState {
public:
    static RobotState& GetInstance() {
        static RobotMotionPauseState instance;
        return instance;
    }
 
    ERESULT StartBuildMap(const std::string& task_id);
private:
    RobotMotionPauseState() {};
    virtual ~RobotMotionPauseState() {};
    RobotMotionPauseState(const RobotMotionPauseState&) {};
    RobotMotionPauseState& operator=(const RobotMotionPauseState&) {};
};

} // end_ns

#endif
