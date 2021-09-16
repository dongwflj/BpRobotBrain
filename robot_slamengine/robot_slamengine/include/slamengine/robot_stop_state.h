/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_STOP_STATE_H_
#define _ROBOT_STOP_STATE_H_

#include "robot_state.h"
#include <string>
using namespace std;

namespace slamengine
{

class RobotStopState : public RobotState {
public:
    static RobotState& getInstance() {
        static RobotStopState instance;
        return instance;
    }
 
    ERESULT StartBuildMap(string task_id); 
private:
    RobotStopState() {};
    virtual ~RobotStopState() {};
    RobotStopState(const RobotStopState&) {};
    RobotStopState& operator=(const RobotStopState&) {};
};

} // end_ns

#endif
