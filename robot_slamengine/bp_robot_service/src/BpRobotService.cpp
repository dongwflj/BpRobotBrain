/*
* Copyright (c) 2021, BytePowers, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/
#include "robot_slamengine.h"
#include "BpRobotService.h"

namespace bp {

BpRobotService::BpRobotService():engine_(RobotSlamEngine::GetInstance())
{
  ROS_INFO("BpRobotService initializing...");
  // Important init engine and ctrl interface
  engine_.Init(robotCtrl_, *this);
  module_nh_ = std::make_shared<ros::NodeHandle>();
  v_subs_.emplace_back(module_nh_->subscribe<std_msgs::Bool>(
                "/EStop", 1, &BpRobotService::EmergencyCallback, this));
  ROS_INFO("BpRobotService Initialization Done");
}

BpRobotService::~BpRobotService()
{
}

ERESULT BpRobotService::OnNaviDone(const std::string& task_id, int32 state, const std::string& description) {
    ROS_INFO("BpRobotService onNaviDone entry");
    ROS_INFO("BpRobotService onNaviDone exit");
    return E_OK;
}

ERESULT BpRobotService::OnNaviActive(const std::string& task_id, bool active) {
    return E_OK;
}

ERESULT BpRobotService::OnNaviProgress(const std::string& task_id, const Pose& curr_pose) {
    return E_OK;
}
 
void BpRobotService::EmergencyCallback(const std_msgs::Bool::ConstPtr &msg) {
    static bool emergency = false;            
    if(!emergency) {
        engine_.NotifyEStopEvent(EMERGENCY_DOWN);
    }else{
        engine_.NotifyEStopEvent(EMERGENCY_UP);
    }

    emergency = msg->data;
}

}

int main(int argc, char** argv)
{ 
    ros::init(argc, argv, "bp_robot_service_node");
    ros::start();
    bp::BpRobotService engine;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}

