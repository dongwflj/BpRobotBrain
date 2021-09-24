/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_ENGINE_OBSERVER_H_
#define _ROBOT_ENGINE_OBSERVER_H_

#include <string>

#include "define.h"

namespace slamengine
{

class IRobotEngineObserver {
public:
    virtual ERESULT OnNaviDone(const std::string& task_id, int32 state, const std::string& description) = 0;
    virtual ERESULT OnNaviActive(const std::string& task_id, bool active) = 0;
    virtual ERESULT OnNaviProgress(const std::string& task_id, const Pose& curr_pose) = 0;
};

} // end_ns

#endif
