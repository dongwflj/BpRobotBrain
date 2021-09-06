#ifndef NAVI_MODULE_H
#define NAVI_MODULE_H

#include <vector>
#include <ros/ros.h>
#include <ros/service.h>
#include <ros/node_handle.h>

#include "ginger_msgs/StartMapping.h"
#include "ginger_msgs/SaveMap.h"
#include "ginger_msgs/BatteryState.h"
#include "ginger_msgs/NaviTo.h"
#include "slamengine/robot_engine_observer.h"
#include "glite_ctrl.h"

namespace ginger
{
using RosNodeHandlePtr = std::shared_ptr<ros::NodeHandle>; 

class NaviModule : public IRobotEngineObserver
{
public:
    NaviModule();
    ~NaviModule();
    // map service
    bool startMappingService(ginger_msgs::StartMappingRequest &req, ginger_msgs::StartMappingResponse &res);
    bool saveMapService(ginger_msgs::SaveMapRequest &req, ginger_msgs::SaveMapResponse &res);
    
    bool naviToService(ginger_msgs::NaviToRequest &req, ginger_msgs::NaviToResponse &res);
    void batteryStateCallback(const ginger_msgs::BatteryState::ConstPtr &msg);
    //IRobotSlamEngineObserver
    ERESULT onNaviDone();
    ERESULT onNaviActive();
    ERESULT onNaviProgress();
private:
    RosNodeHandlePtr module_nh_;
    GliteCtrl gliteCtrl_;
    RobotSlamEngine& engine_;
    std::vector<ros::ServiceServer> v_servers_;
    std::vector<ros::Subscriber> v_subs_;
};

}

#endif
