/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_CTRL_H_
#define _ROBOT_CTRL_H_

#include "define.h"
#include <string>
using namespace std;

namespace slamengine
{

class IRobotObserver;

class IRobotCtrl {
public:
    virtual ERESULT StartBuildMap(string task_id) = 0;
    virtual ERESULT StopBuildMap(string task_id) = 0;
    virtual ERESULT PauseBuildMap(string task_id) = 0;
    virtual ERESULT ResumeBuildMap(string task_id) = 0;
    virtual ERESULT SaveMap(string task_id) = 0;
    virtual ERESULT LoadMap(string task_id) = 0;  

    virtual ERESULT StartNavi(string task_id, ENAVITYPE type, string goal_name, PixelPose goal_pose) = 0;
    virtual ERESULT StopNavi(string task_id) = 0;
    virtual ERESULT setObserver(IRobotObserver&) = 0;
};

} // end_ns

#endif
