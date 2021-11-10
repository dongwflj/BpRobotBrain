/*
* Copyright (c) 2021, BytePowers, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/
#ifndef BP_ROBOT_SERVICE_H
#define BP_ROBOT_SERVICE_H

#include <vector>
#include <ros/ros.h>
#include <ros/service.h>
#include <ros/node_handle.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_srvs/SetBool.h>
#include "robot_engine_observer.h"
#include "BpRobotCtrl.h"

namespace bp
{
using RosNodeHandlePtr = std::shared_ptr<ros::NodeHandle>; 

class BpRobotService : public IRobotEngineObserver
{
public:
    BpRobotService();
    ~BpRobotService();
    void EmergencyCallback(const std_msgs::Bool::ConstPtr &msg);

    ERESULT OnNaviDone(const std::string& task_id, int32 state, const std::string& description);
    ERESULT OnNaviActive(const std::string& task_id, bool active);
    ERESULT OnNaviProgress(const std::string& task_id, const Pose& curr_pose);

private:
    RosNodeHandlePtr module_nh_;
    BpRobotCtrl robotCtrl_;
    RobotSlamEngine& engine_;
    std::vector<ros::ServiceServer> v_servers_;
    std::vector<ros::Subscriber> v_subs_;
};

}

#endif
