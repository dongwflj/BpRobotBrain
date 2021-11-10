/*
* Copyright (c) 2021, BytePowers, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/
#ifndef _BP_ROBOT_CTRL_H_
#define _BP_ROBOT_CTRL_H_

#include <ros/ros.h> 
#include "robot_ctrl.h"

namespace bp {
class IRobotObserver;
class BpRobotCtrl : public IRobotCtrl {
public:
    BpRobotCtrl();
    ~BpRobotCtrl();
    ERESULT Init();
    ERESULT StartBuildMap(const std::string& task_id);
    ERESULT StopBuildMap(const std::string& task_id);
    ERESULT CancelBuildMap(const std::string& task_id);
    ERESULT PauseBuildMap(const std::string& task_id);
    ERESULT ResumeBuildMap(const std::string& task_id);
    ERESULT SaveMap(const std::string& task_id, const std::string& map_name);
    ERESULT LoadMap(const std::string& task_id, const std::string& map_name);  

    ERESULT StartNavi(const std::string& task_id, ENAVITYPE type, const std::string& goal_name, const PixelPose& goal_pose);
    ERESULT StopNavi(const std::string& task_id);

    ERESULT CheckPose(const std::string& poi_name, const PixelPose& poi_pose);
    ERESULT SetPixelPose(const PixelPose& pixel_pose);
    ERESULT SetMaxNaviVel(double max_linear_vel, double max_angular_max);

    ERESULT SetObserver(IRobotObserver&);
private:
    IRobotObserver *observer_;
};

}

#endif
