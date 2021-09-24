/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/
#ifndef _GLITE_CTRL_H_
#define _GLITE_CTRL_H_

#include "robot_ctrl.h"

namespace slamengine {
class IRobotObserver;
}

namespace ginger
{
using namespace slamengine;
class PatrolCtrl : public IRobotCtrl
{
public:
    ERESULT Init();
    ERESULT StartBuildMap(const std::string& task_id);
    ERESULT StopBuildMap(const std::string& task_id);
    ERESULT PauseBuildMap(const std::string& task_id);
    ERESULT ResumeBuildMap(const std::string& task_id);
    ERESULT SaveMap(const std::string& task_id, const std::string& map_name);
    ERESULT LoadMap(const std::string& task_id, const std::string& map_name);  

    ERESULT StartNavi(const std::string& task_id, ENAVITYPE type, const std::string& goal_name, const PixelPose& goal_pose);
    ERESULT StopNavi(const std::string& task_id) {};

    ERESULT SetObserver(IRobotObserver&);
private:
    IRobotObserver *observer_;
};

}

#endif
