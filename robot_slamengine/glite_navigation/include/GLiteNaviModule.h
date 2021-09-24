#ifndef NAVI_MODULE_H
#define NAVI_MODULE_H

#include <vector>
#include <ros/ros.h>
#include <ros/service.h>
#include <ros/node_handle.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_srvs/SetBool.h>
#include "ginger_msgs/StartMapping.h"
#include "ginger_msgs/LoadMap.h"
#include "ginger_msgs/SaveMap.h"
#include "ginger_msgs/CheckPose.h"
#include "ginger_msgs/SetPose.h"
#include "ginger_msgs/SetWorldPose.h"
#include "ginger_msgs/BatteryState.h"
#include "ginger_msgs/NaviTo.h"
#include "ginger_msgs/NaviVel.h"
#include "ginger_msgs/NaviCmd.h"
#include "ginger_msgs/BatteryState.h"
#include "ginger_msgs/DropCollision.h"
#include "ginger_msgs/NaviCmd.h"
#include "ginger_msgs/NaviState.h"
#include "ginger_msgs/NaviTo.h"
#include "ginger_msgs/PixelPose.h"

#include "robot_engine_observer.h"
#include "GLiteCtrl.h"

namespace ginger
{
using RosNodeHandlePtr = std::shared_ptr<ros::NodeHandle>; 

class GLiteNaviModule : public IRobotEngineObserver
{
public:
    GLiteNaviModule();
    ~GLiteNaviModule();
    // map service
    bool startMappingService(ginger_msgs::StartMappingRequest &req, ginger_msgs::StartMappingResponse &res);
    bool saveMapService(ginger_msgs::SaveMapRequest &req, ginger_msgs::SaveMapResponse &res);
    bool loadMapService(ginger_msgs::LoadMapRequest &req, ginger_msgs::LoadMapResponse &res);
    bool checkPoseService(ginger_msgs::CheckPoseRequest &req, ginger_msgs::CheckPoseResponse &res);
    bool setPoseService(ginger_msgs::SetPoseRequest &req, ginger_msgs::SetPoseResponse &res);
    bool setWorldPoseService(ginger_msgs::SetWorldPoseRequest &req, ginger_msgs::SetWorldPoseResponse &res);
    bool naviToService(ginger_msgs::NaviToRequest &req, ginger_msgs::NaviToResponse &res);
    bool naviCmdService(ginger_msgs::NaviCmdRequest &req, ginger_msgs::NaviCmdResponse &res);
    bool naviVelService(ginger_msgs::NaviVelRequest &req, ginger_msgs::NaviVelResponse &res);

    void DropCollisionCallback(const ginger_msgs::DropCollision::ConstPtr &msg); 
    void EmergencyCallback(const std_msgs::Bool::ConstPtr &msg);
    void BatteryStateCallback(const ginger_msgs::BatteryState::ConstPtr &msg);

    ERESULT OnNaviDone(const std::string& task_id, int32 state, const std::string& description);
    ERESULT OnNaviActive(const std::string& task_id, bool active);
    ERESULT OnNaviProgress(const std::string& task_id, const Pose& curr_pose);

private:
    RosNodeHandlePtr module_nh_;
    GliteCtrl gliteCtrl_;
    RobotSlamEngine& engine_;
    std::vector<ros::ServiceServer> v_servers_;
    std::vector<ros::Subscriber> v_subs_;
};

}

#endif
