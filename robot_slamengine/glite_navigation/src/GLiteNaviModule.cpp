#include "robot_slamengine.h"
#include "GLiteNaviModule.h"


namespace ginger {

GLiteNaviModule::GLiteNaviModule():engine_(RobotSlamEngine::GetInstance())
{
  ROS_INFO("GLiteNaviModule initializing...");
  // Important init engine and ctrl interface
  engine_.Init(gliteCtrl_, *this);
  module_nh_ = std::make_shared<ros::NodeHandle>();
  v_servers_.push_back(module_nh_->advertiseService("StartMapping", &GLiteNaviModule::startMappingService, this));
  v_servers_.push_back(module_nh_->advertiseService("SaveMap", &GLiteNaviModule::saveMapService, this));
  v_servers_.push_back(module_nh_->advertiseService("LoadMap", &GLiteNaviModule::loadMapService, this));
  v_servers_.push_back(module_nh_->advertiseService("CheckPose", &GLiteNaviModule::checkPoseService, this));
  v_servers_.push_back(module_nh_->advertiseService("SetPose", &GLiteNaviModule::setPoseService, this));
  v_servers_.push_back(module_nh_->advertiseService("SetWorldPose", &GLiteNaviModule::setWorldPoseService, this));
  v_servers_.push_back(module_nh_->advertiseService("NaviTo", &GLiteNaviModule::naviToService, this));
  v_servers_.push_back(module_nh_->advertiseService("NaviCmd", &GLiteNaviModule::naviCmdService, this));
  v_servers_.push_back(module_nh_->advertiseService("NaviVel", &GLiteNaviModule::naviVelService, this));

  v_subs_.emplace_back(module_nh_->subscribe<ginger_msgs::DropCollision>(
                "/DropCollision", 1, &GLiteNaviModule::DropCollisionCallback, this));
  v_subs_.emplace_back(module_nh_->subscribe<std_msgs::Bool>(
                "/EStop", 1, &GLiteNaviModule::EmergencyCallback, this));
  v_subs_.emplace_back(module_nh_->subscribe<ginger_msgs::BatteryState>(
                "/BatteryState", 1, &GLiteNaviModule::BatteryStateCallback, this));
  ROS_INFO("GLiteNaviModule Initialization Done");
}

GLiteNaviModule::~GLiteNaviModule()
{
}

ERESULT GLiteNaviModule::OnNaviDone(const std::string& task_id, int32 state, const std::string& description) {
    ROS_INFO("GLiteNaviModule onNaviDone entry");
    ROS_INFO("GLiteNaviModule onNaviDone exit");
    return E_OK;
}

ERESULT GLiteNaviModule::OnNaviActive(const std::string& task_id, bool active) {
    return E_OK;
}

ERESULT GLiteNaviModule::OnNaviProgress(const std::string& task_id, const Pose& curr_pose) {
    return E_OK;
}
 
bool GLiteNaviModule::startMappingService(ginger_msgs::StartMappingRequest& req, ginger_msgs::StartMappingResponse& res)
{
    ROS_INFO("GLiteNaviModule: start calling mapping service");
    ERESULT result = engine_.StartBuildMap("");
    res.result = result;
    
    switch(res.result){
        case E_BUILDMAPSUCCESS:
        res.description = "Mapping started successfully";
        break;
        case E_BUILDMAPFAILED:
        res.description = "Mapping started failed";
        break;
        case E_BUILDMAPREPEAT:
        res.description = "Mapping has been started, Don't Repeat";
        break;
    }

    return true;
}

bool GLiteNaviModule::saveMapService(ginger_msgs::SaveMapRequest &req, ginger_msgs::SaveMapResponse &res)
{
    ROS_INFO("GLiteNaviModule: save map service");
    ERESULT result = engine_.SaveMap("", req.name);
    res.result = result;

    switch(res.result){
        case E_SAVEMAP_SUCCESS:
        res.description = "Save map successfully";
        break;
        case E_SAVEMAP_FAILED:
        res.description = "Save map failed";
        break;
        case E_SAVEMAP_MAPPING_NOT_START:
        res.description = "Cannot save map, because mapping not start";
        break;
        case E_SAVEMAP_PATH_UNVALID:
        res.description = "Cannot save map, because map path is invalid";
        break;
    }
    return true;
}

bool GLiteNaviModule::loadMapService(ginger_msgs::LoadMapRequest &req, ginger_msgs::LoadMapResponse &res) {
    ROS_INFO("GLiteNaviModule: load map service");
    ERESULT result = engine_.LoadMap("", req.name);
    res.result = result;

    switch(res.result){
        case E_LOADMAP_SUCCESS:
        res.description = "Load map successfully";
        break;
        case E_LOADMAP_FAILED:
        res.description = "Load map failed";
        break;
        case E_LOADMAP_MAPPING_NOT_START:
        res.description = "Cannot load map, because mapping not start";
        break;
        case E_LOADMAP_PATH_UNVALID:
        res.description = "Cannot load map, because map path is invalid";
        break;
    }
    return true;
}

bool GLiteNaviModule::checkPoseService(ginger_msgs::CheckPoseRequest &req, ginger_msgs::CheckPoseResponse &res){
#if 0
    ROS_INFO("GLiteNaviModule: check pose service");
    ERESULT result = engine_.LoadMap("", req.name);
    res.result = result;

    switch(res.result){
        case E_LOADMAP_SUCCESS:
        res.description = "Load map successfully";
        break;
        case E_LOADMAP_FAILED:
        res.description = "Load map failed";
        break;
        case E_LOADMAP_MAPPING_NOT_START:
        res.description = "Cannot load map, because mapping not start";
        break;
        case E_LOADMAP_PATH_UNVALID:
        res.description = "Cannot load map, because map path is invalid";
        break;
    }
#endif

    return true;
}

bool GLiteNaviModule::setPoseService(ginger_msgs::SetPoseRequest &req, ginger_msgs::SetPoseResponse &res){
    return true;
}

bool GLiteNaviModule::setWorldPoseService(ginger_msgs::SetWorldPoseRequest &req, ginger_msgs::SetWorldPoseResponse &res){
    return true;
}

bool GLiteNaviModule::naviToService(ginger_msgs::NaviToRequest &req, ginger_msgs::NaviToResponse &res)
{
    PixelPose pose;
    pose.x = req.pose.x;
    pose.y = req.pose.y;
    pose.theta = req.pose.theta; 

    ENAVITYPE type;
    switch(req.type){
        case ginger_msgs::NaviToRequest::NORMAL_NAVI:
        type = NAVI_NORMAL; 
        break;
        case ginger_msgs::NaviToRequest::LIFT_NAVI:
        type = NAVI_GOINLIFT;
        break;
        case ginger_msgs::NaviToRequest::LIFT_NAVI_OUT:
        type = NAVI_GOOUTLIFT;
        break;
        case ginger_msgs::NaviToRequest::CHARGE_NAVI:
        type = NAVI_GOCHARGE;
        break;
    } 

    ERESULT result = engine_.StartNavi(req.task_id, type, req.name, pose);
    res.result = result;
    switch(res.result){
        case E_LOADMAP_SUCCESS:
        res.description = "Load map successfully";
        break;
        case E_LOADMAP_FAILED:
        res.description = "Load map failed";
        break;
        case E_LOADMAP_MAPPING_NOT_START:
        res.description = "Cannot load map, because mapping not start";
        break;
        case E_LOADMAP_PATH_UNVALID:
        res.description = "Cannot load map, because map path is invalid";
        break;
    }

    return true;
}

bool GLiteNaviModule::naviCmdService(ginger_msgs::NaviCmdRequest &req, ginger_msgs::NaviCmdResponse &res){
    return true;
}

bool GLiteNaviModule::naviVelService(ginger_msgs::NaviVelRequest &req, ginger_msgs::NaviVelResponse &res){
    return true;
}

void GLiteNaviModule::DropCollisionCallback(const ginger_msgs::DropCollision::ConstPtr &msg) {
    static bool drop_head = false;
    static bool drop_back = false;
    static bool collision = false;

    if (msg->isDrop[0] && !drop_head) {
        drop_head = msg->isDrop[0];
        ROS_DEBUG("GingerNavi: cliff detected ahead");
        engine_.NotifyDropEvent(CLIFF_DETECTED_AHEAD); 
    } else if (!msg->isDrop[0] && drop_head) {
        drop_head = msg->isDrop[0];
        ROS_DEBUG("GingerNavi: cliff resumed ahead");
        engine_.NotifyDropEvent(CLIFF_RESUMED_AHEAD); 
    }

    if (msg->isDrop[1] && !drop_back) {
        drop_back = msg->isDrop[1];
        ROS_DEBUG("GingerNavi: cliff detected back");
        engine_.NotifyDropEvent(CLIFF_DETECTED_BACK); 
    } else if (!msg->isDrop[1] && drop_back) {
        drop_back = msg->isDrop[1];
        ROS_DEBUG("GingerNavi: cliff resumed back");
        engine_.NotifyDropEvent(CLIFF_RESUMED_BACK); 
    }

    if (msg->isCollision[0] && !collision) {
        collision = msg->isCollision[0];
        ROS_DEBUG("GingerNavi: bump detected");
        engine_.NotifyCollisionEvent(COLLISION_DETECTED); 
    } else if (!msg->isCollision[0] && collision) {
        collision = msg->isCollision[0];
        ROS_DEBUG("GingerNavi: bump clear");
        engine_.NotifyCollisionEvent(COLLISION_RESUMED); 
    }
}

void GLiteNaviModule::EmergencyCallback(const std_msgs::Bool::ConstPtr &msg) {
    static bool emergency = false;            
    if(!emergency) {
        engine_.NotifyEStopEvent(EMERGENCY_DOWN);
    }else{
        engine_.NotifyEStopEvent(EMERGENCY_UP);
    }

    emergency = msg->data;
}

void GLiteNaviModule::BatteryStateCallback(
    const ginger_msgs::BatteryState::ConstPtr &msg) {
    BatteryState battery_state;
    battery_state.voltage = msg->voltage;
    battery_state.current = msg->current;
    battery_state.current_capacity = msg->current_capacity;
    battery_state.init_capacity = msg->init_capacity;
    battery_state.dump_energy = msg->dump_energy;
    battery_state.cell_type = msg->cell_type;
    battery_state.id = msg->id;
    battery_state.status = msg->status;
    battery_state.chargeset_type = msg->chargeset_type;

    engine_.NotifyChargingEvent(battery_state);
}

}//end of ns

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

