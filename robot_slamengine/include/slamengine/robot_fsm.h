/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_FSM_H_
#define _ROBOT_FSM_H_

namespace slamengine
{
class RobotState;
class IRobotCtrl;
class IRobotObserver;

class IRobotFsm {
public:
    virtual void transitionTo(RobotState *state) = 0; 
    virtual IRobotCtrl& getRobotCtrl() = 0; 
    virtual IRobotObserver& getRobotObserver() = 0; 
};

} // end_ns

#endif
