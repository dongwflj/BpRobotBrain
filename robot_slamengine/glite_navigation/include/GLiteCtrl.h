/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/
#ifndef _GLITE_CTRL_H_
#define _GLITE_CTRL_H_

#include "slamengine/robot_ctrl.h"

namespace slamengine {
class IRobotObserver;
}

namespace ginger
{
using namespace slamengine;
class GliteCtrl : public IRobotCtrl
{
public:
    ERESULT StartBuildMap(string task_id);
    ERESULT StopBuildMap(string task_id);
    ERESULT PauseBuildMap(string task_id);
    ERESULT ResumeBuildMap(string task_id);
    ERESULT SaveMap(string task_id);
    ERESULT LoadMap(string task_id);  

    ERESULT StartNavi(string task_id, ENAVITYPE type, string goal_name, PixelPose goal_pose);
    ERESULT StopNavi(string task_id) {};

    ERESULT setObserver(IRobotObserver&);
private:
    IRobotObserver *observer_;
};

}

#endif
