/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_STATE_H_
#define _ROBOT_STATE_H_

#include "define.h"
#include <string>
using namespace std;

namespace slamengine
{
class RobotFsm;
class RobotState {
public:
    virtual ~RobotState() {}
    void set_context(RobotFsm *context) {
        this->context_ = context;
    }

    virtual ERESULT StartBuildMap(string task_id);
    virtual ERESULT StopBuildMap(string task_id);
    virtual ERESULT PauseBuildMap(string task_id);
    virtual ERESULT ResumeBuildMap(string task_id);
    
    // Navi
    virtual ERESULT StartNavi(string task_id, ENAVITYPE type, string goal_name, PixelPose goal_pose);
    virtual ERESULT StopNavi(string task_id);
    virtual ERESULT PauseNavi(string task_id);
    virtual ERESULT ResumeNavi(string task_id);

    virtual ERESULT NaviDoneEvent(string task_id, int32 state, string description);
    virtual ERESULT NaviActiveEvent(string task_id, bool active);
    virtual ERESULT NaviProgressEvent(string task_id, Pose curr_pose);
protected:
    RobotFsm *context_;
};

} // end_ns

#endif
