/*
 * File: NaviModule.h
 * Project: ginger_navigation
 * Created Date: Thursday, December 12th 2019, 3:53:23 pm
 * Author: Chen Hao (hao.chen@cloudminds.com)
 * -----
 * Last Modified: Mon Oct 12 2020
 * Modified By: Chen Hao
 * -----
 * Copyright (c) 2019 CloudMinds
 *
 * Good Good Study, Day Day Up
 * ----------------------------------------------------------
 */

#ifndef NAVI_MODULE_H
#define NAVI_MODULE_H

#include <vector>
#include <ros/ros.h>
#include <ros/service.h>
#include <ros/node_handle.h>

#include "ginger_msgs/StartMapping.h"
#include "ginger_msgs/SaveMap.h"
#include "ginger_msgs/BatteryState.h"
#include "glite_ctrl.h"
#include "slamengine/robot_observer.h"

/*
#include <thread>

#include <jsoncpp/json/json.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Time.h>
#include <std_msgs/Int8.h>

#include "ginger_navigation/GingerMap.h"
#include "ginger_navigation/GingerNavi.h"
#include "ginger_msgs/StartMapping.h"
#include "ginger_msgs/LoadMap.h"
#include "ginger_msgs/CheckPose.h"
#include "ginger_msgs/SetPose.h"
#include "ginger_msgs/SetWorldPose.h"
#include "ginger_msgs/DockState.h"
#include "ginger_msgs/BatteryState.h"
#include "ginger_msgs/RdmAlarm.h"
#include "ginger_msgs/NaviVel.h"
#include "ginger_msgs/GrpcCommonResp.h"
#include "ginger_msgs/SetSlamModel.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include "yaml-cpp/yaml.h"
*/
namespace slamengine {
class RobotSlamEngine;
}

namespace ginger
{
using RosNodeHandlePtr = std::shared_ptr<ros::NodeHandle>; 
class NaviModule : public IRobotObserver
{
public:
    NaviModule();
    ~NaviModule();
    // map service
    bool startMappingService(ginger_msgs::StartMappingRequest &req, ginger_msgs::StartMappingResponse &res);
    bool saveMapService(ginger_msgs::SaveMapRequest &req, ginger_msgs::SaveMapResponse &res);
    
    void batteryStateCallback(const ginger_msgs::BatteryState::ConstPtr &msg);
/*
    bool loadMapService(ginger_msgs::LoadMapRequest &req, ginger_msgs::LoadMapResponse &res);
    bool checkPoseService(ginger_msgs::CheckPoseRequest &req, ginger_msgs::CheckPoseResponse &res);
    bool setPoseService(ginger_msgs::SetPoseRequest &req, ginger_msgs::SetPoseResponse &res);
    bool setWorldPoseService(ginger_msgs::SetWorldPoseRequest &req, ginger_msgs::SetWorldPoseResponse &res);

    // navi service
    bool naviToService(ginger_msgs::NaviToRequest &req, ginger_msgs::NaviToResponse &res);
    bool naviCmdService(ginger_msgs::NaviCmdRequest &req, ginger_msgs::NaviCmdResponse &res);
    bool naviVelService(ginger_msgs::NaviVelRequest &req, ginger_msgs::NaviVelResponse &res);

    bool ChassisParamConfigCallback(ginger_msgs::GrpcCommonResp::Request &req,
                                    ginger_msgs::GrpcCommonResp::Response &res);
                                    */
    //IRobotObserver
    ERESULT onNaviDone();
    ERESULT onNaviActive();
    ERESULT onNaviProgress();
 
private:
    RosNodeHandlePtr module_nh_;
    GliteCtrl gliteCtrl_;
    slamengine::RobotSlamEngine& engine_;
    std::vector<ros::ServiceServer> v_servers_;
    std::vector<ros::Subscriber> v_subs_;
};


}

#endif
