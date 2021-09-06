#include "slamengine/robot_slamengine.h"
#include "NaviModule.h"


namespace ginger {

NaviModule::NaviModule():engine_(RobotSlamEngine::getInstance())
{
  ROS_INFO("NaviModule initializing...");
  // Important init engine and ctrl interface
  engine_.init(gliteCtrl_, *this);
  module_nh_ = std::make_shared<ros::NodeHandle>();
  v_servers_.push_back(module_nh_->advertiseService("StartMapping", &NaviModule::startMappingService, this));
  v_servers_.push_back(module_nh_->advertiseService("SaveMap", &NaviModule::saveMapService, this));
  v_servers_.push_back(module_nh_->advertiseService("NaviTo", &NaviModule::naviToService, this));
  v_subs_.emplace_back(module_nh_->subscribe<ginger_msgs::BatteryState>(
                "/BatteryState", 1, &NaviModule::batteryStateCallback, this));
  ROS_INFO("NaviModule Initialization Done");
}

NaviModule::~NaviModule()
{
}

ERESULT NaviModule::onNaviDone() {
    ROS_INFO("NaviModule onNaviDone entry");
    ROS_INFO("NaviModule onNaviDone exit");
    return E_OK;
}

ERESULT NaviModule::onNaviActive() {
    return E_OK;
}

ERESULT NaviModule::onNaviProgress() {
    return E_OK;
}
 
bool NaviModule::startMappingService(ginger_msgs::StartMappingRequest &req, ginger_msgs::StartMappingResponse &res)
{
    ROS_INFO("NaviModule: start calling mapping service");
    ERESULT result = engine_.startBuildMap();
    res.result = result;
    res.description = "Mapping Has Been Started, Don't Repeat";
    return true;
}

bool NaviModule::saveMapService(ginger_msgs::SaveMapRequest &req, ginger_msgs::SaveMapResponse &res)
{
    ROS_INFO("NaviModule: save map service");
    ERESULT result = engine_.stopBuildMap();
    res.result = result;
    res.description = "Cannot save map, because mapping not start";
    return true;
}

bool NaviModule::naviToService(ginger_msgs::NaviToRequest &req, ginger_msgs::NaviToResponse &res)
{
    ERESULT result = engine_.startNavi(NAVI_NORMAL);
    res.result = result;
    res.description = "Start navi ok";
    return true;
}

void NaviModule::batteryStateCallback(
    const ginger_msgs::BatteryState::ConstPtr &msg) {
	engine_.notifyChargingEvent();
}

}

int main(int argc, char** argv)
{ 
    ros::init(argc, argv, "robot_slamengine_nonde");
    ros::start();
    ginger::NaviModule engine;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
