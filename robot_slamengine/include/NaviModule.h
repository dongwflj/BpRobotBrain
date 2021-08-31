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
#include "ginger_msgs/SaveMap.h"
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
namespace ginger
{
using RosNodeHandlePtr = std::shared_ptr<ros::NodeHandle>; 
class NaviModule
{
public:
    NaviModule();
    ~NaviModule();
    // map service
    bool startMappingService(ginger_msgs::StartMappingRequest &req, ginger_msgs::StartMappingResponse &res);
/*
    bool saveMapService(ginger_msgs::SaveMapRequest &req, ginger_msgs::SaveMapResponse &res);
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
private:
    int loadMapImpl(std::string map_name);

    RosNodeHandlePtr map_nh_;
    RosNodeHandlePtr navi_nh_;
    RosNodeHandlePtr module_nh_;
    RosNodeHandlePtr param_nh_;
    std::vector<ros::ServiceServer> v_servers_;
    ros::ServiceClient slam_model_client_;
    std::shared_ptr<ros::Publisher> rdm_pub_ = nullptr;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_ = nullptr;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_cast_ = nullptr;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_ = nullptr;

    std::shared_ptr<GingerMap> p_map_ = nullptr;
    std::shared_ptr<GingerNavi> p_navi_ = nullptr;
    double d_gearSpeed[3];
    double d_turnSpeed[3];
    int i_gearSpeed_curr;
    int i_turnSpeed_curr;
    geometry_msgs::PoseStamped cur_lift_in_pose_;
    void resetRobotPose();
    bool navi_vel_set();
    int ParamUpdate();

    double harix_max_linear_vel_ = 0.0;
    double harix_max_rot_vel_ = 0.0;
    */
    std::vector<ros::ServiceServer> v_servers_;
    RosNodeHandlePtr module_nh_;
};


}

#endif
