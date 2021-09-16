#include "slamengine/robot_slamengine.h"
#include "GLiteNaviModule.h"


namespace ginger {

GLiteNaviModule::GLiteNaviModule():engine_(RobotSlamEngine::getInstance())
{
  ROS_INFO("GLiteNaviModule initializing...");
  // Important init engine and ctrl interface
  engine_.Init(gliteCtrl_, *this);
  module_nh_ = std::make_shared<ros::NodeHandle>();
  v_servers_.push_back(module_nh_->advertiseService("StartMapping", &GLiteNaviModule::startMappingService, this));
  v_servers_.push_back(module_nh_->advertiseService("SaveMap", &GLiteNaviModule::saveMapService, this));
  v_servers_.push_back(module_nh_->advertiseService("NaviTo", &GLiteNaviModule::naviToService, this));
  v_subs_.emplace_back(module_nh_->subscribe<ginger_msgs::BatteryState>(
                "/BatteryState", 1, &GLiteNaviModule::batteryStateCallback, this));
  ROS_INFO("GLiteNaviModule Initialization Done");
}

GLiteNaviModule::~GLiteNaviModule()
{
}

ERESULT GLiteNaviModule::onNaviDone(string task_id, int32 state, string description) {
    ROS_INFO("GLiteNaviModule onNaviDone entry");
    ROS_INFO("GLiteNaviModule onNaviDone exit");
    return E_OK;
}

ERESULT GLiteNaviModule::onNaviActive(string task_id, bool active) {
    return E_OK;
}

ERESULT GLiteNaviModule::onNaviProgress(string task_id, Pose curr_pose) {
    return E_OK;
}
 
bool GLiteNaviModule::startMappingService(ginger_msgs::StartMappingRequest &req, ginger_msgs::StartMappingResponse &res)
{
    ROS_INFO("GLiteNaviModule: start calling mapping service");
    ERESULT result = engine_.StartBuildMap("");
    res.result = result;
    res.description = "Mapping Has Been Started, Don't Repeat";
    return true;
}

bool GLiteNaviModule::saveMapService(ginger_msgs::SaveMapRequest &req, ginger_msgs::SaveMapResponse &res)
{
    ROS_INFO("GLiteNaviModule: save map service");
    ERESULT result = engine_.StopBuildMap("");
    res.result = result;
    res.description = "Cannot save map, because mapping not start";
    return true;
}

bool GLiteNaviModule::naviToService(ginger_msgs::NaviToRequest &req, ginger_msgs::NaviToResponse &res)
{
    PixelPose pose;
    pose.x = req.pose.x;
    pose.y = req.pose.y;
    pose.theta = req.pose.theta; 
    ERESULT result = engine_.StartNavi(req.task_id, NAVI_NORMAL, req.name, pose);
    res.result = result;
    res.description = "Start navi ok";
    return true;
}

void GLiteNaviModule::batteryStateCallback(
    const ginger_msgs::BatteryState::ConstPtr &msg) {
	//engine_.notifyChargingEvent();
}

}

int main(int argc, char** argv)
{ 
    ros::init(argc, argv, "robot_slamengine_nonde");
    ros::start();
    ginger::GLiteNaviModule engine;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
