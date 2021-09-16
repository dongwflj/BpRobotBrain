/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_ENGINE_OBSERVER_H_
#define _ROBOT_ENGINE_OBSERVER_H_

#include "define.h"
#include <string>
using namespace std;

namespace slamengine
{

class IRobotEngineObserver {
public:
    virtual ERESULT onNaviDone(string task_id, int32 state, string description) = 0;
    virtual ERESULT onNaviActive(string task_id, bool active) = 0;
    virtual ERESULT onNaviProgress(string task_id, Pose curr_pose) = 0;
};

} // end_ns

#endif
