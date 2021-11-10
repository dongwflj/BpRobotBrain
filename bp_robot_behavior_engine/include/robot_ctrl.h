/*
* Copyright (c) 2021, BytePowers, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_CTRL_H_
#define _ROBOT_CTRL_H_

#include <string>

#include "define.h"

namespace bp
{

class IRobotObserver;

class IRobotCtrl {
public:
    virtual ERESULT Init() = 0;
    virtual ERESULT StartBuildMap(const std::string& task_id) = 0;
    virtual ERESULT StopBuildMap(const std::string& task_id) = 0;
    virtual ERESULT PauseBuildMap(const std::string& task_id) = 0;
    virtual ERESULT ResumeBuildMap(const std::string& task_id) = 0;
    virtual ERESULT SaveMap(const std::string& task_id, const std::string& map_name) = 0;
    virtual ERESULT LoadMap(const std::string& task_id, const std::string& map_name) = 0;  

    virtual ERESULT StartNavi(const std::string& task_id, ENAVITYPE type, const std::string& goal_name, const PixelPose& goal_pose) = 0;
    virtual ERESULT StopNavi(const std::string& task_id) = 0;
    virtual ERESULT SetObserver(IRobotObserver&) = 0;
};

} // end_ns

#endif
