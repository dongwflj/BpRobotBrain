/*
 * File: NaviModule.cpp
 * Project: ginger_navigation
 * Created Date: Thursday, December 12th 2019, 9:44:43 pm
 * Author: Chen Hao (hao.chen@cloudminds.com)
 * -----
 * Last Modified: Tue Oct 20 2020
 * Modified By: Chen Hao
 * -----
 * Copyright (c) 2019 CloudMinds
 *
 * Good Good Study, Day Day Up
 * ----------------------------------------------------------
 */

#include "slamengine/robot_slamengine.h"
#include "NaviModule.h"

namespace ginger
{

NaviModule::NaviModule()
{
  ROS_INFO("NaviModule initializing...");
  module_nh_ = std::make_shared<ros::NodeHandle>();
  v_servers_.push_back(module_nh_->advertiseService("StartMapping", &NaviModule::startMappingService, this));
  RobotSlamEngine engin = RobotSlamEngine::getInstance();
/*
  ros::NodeHandle nh;
  nh.getParam("/moveSpeed/gearSpeed/high", d_gearSpeed[0]);
  nh.getParam("/moveSpeed/gearSpeed/middle", d_gearSpeed[1]);
  nh.getParam("/moveSpeed/gearSpeed/low", d_gearSpeed[2]);
  nh.getParam("/moveSpeed/turnSpeed/high", d_turnSpeed[0]);
  nh.getParam("/moveSpeed/turnSpeed/middle", d_turnSpeed[1]);
  nh.getParam("/moveSpeed/turnSpeed/low", d_turnSpeed[2]);
  nh.getParam("/moveSpeedHari/gearSpeedCurr", i_gearSpeed_curr);
  nh.getParam("/moveSpeedHari/turnSpeedCurr", i_turnSpeed_curr);

  if (i_gearSpeed_curr < 1 || i_gearSpeed_curr > 3) {
    i_gearSpeed_curr = 1;
    ROS_ERROR(
        "Navi: gearSpeedCurr abnormal, please check "
        "~/ginlt_param/robot_config_hari.yaml");
  }
  else {
      harix_max_linear_vel_ = d_gearSpeed[i_gearSpeed_curr - 1];
  }

  if (i_turnSpeed_curr < 1 || i_turnSpeed_curr > 3) {
    i_turnSpeed_curr = 2;
    ROS_ERROR(
        "Navi: i_turnSpeed_curr abnormal, please check "
        "~/ginlt_param/robot_config_hari.yaml");
  }
  else {
      harix_max_rot_vel_ = d_turnSpeed[i_turnSpeed_curr - 1];
  }

  ROS_INFO("gearSpeed: high %.4f, middle %.4f, low %.4f", d_gearSpeed[0],
           d_gearSpeed[1], d_gearSpeed[2]);
  ROS_INFO("turnSpeed: high %.4f, middle %.4f, low %.4f", d_turnSpeed[0],
           d_turnSpeed[1], d_turnSpeed[2]);
  ROS_INFO("moveSpeedHari: gearSpeedCurr %d, turnSpeedCurr %d",
           i_gearSpeed_curr, i_turnSpeed_curr);
  ROS_INFO("Navi: p_navi_->setVel gearSpeed[%d] %.2f, turnSpeed[%d] %.2f",
           i_gearSpeed_curr - 1, d_gearSpeed[i_gearSpeed_curr - 1],
           i_turnSpeed_curr - 1, d_turnSpeed[i_turnSpeed_curr - 1]);

    map_nh_ = std::make_shared<ros::NodeHandle>();
    navi_nh_ = std::make_shared<ros::NodeHandle>();
    module_nh_ = std::make_shared<ros::NodeHandle>();
    param_nh_ = std::make_shared<ros::NodeHandle>();

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf2_cast_ = std::make_shared<tf2_ros::TransformBroadcaster>();
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    rdm_pub_ = std::make_shared<ros::Publisher>(module_nh_->advertise<ginger_msgs::RdmAlarm>("rdm_alarm", 1, true));

    //ROS service
    v_servers_.push_back(module_nh_->advertiseService("StartMapping", &NaviModule::startMappingService, this));
    v_servers_.push_back(module_nh_->advertiseService("SaveMap", &NaviModule::saveMapService, this));
    v_servers_.push_back(module_nh_->advertiseService("LoadMap", &NaviModule::loadMapService, this));
    v_servers_.push_back(module_nh_->advertiseService("CheckPose", &NaviModule::checkPoseService, this));
    v_servers_.push_back(module_nh_->advertiseService("SetPose", &NaviModule::setPoseService, this));
    v_servers_.push_back(module_nh_->advertiseService("SetWorldPose", &NaviModule::setWorldPoseService, this));
    v_servers_.push_back(module_nh_->advertiseService("NaviTo", &NaviModule::naviToService, this));
    v_servers_.push_back(module_nh_->advertiseService("NaviCmd", &NaviModule::naviCmdService, this));
    v_servers_.push_back(module_nh_->advertiseService("NaviVel", &NaviModule::naviVelService, this));
    v_servers_.push_back(module_nh_->advertiseService("configMove", &NaviModule::ChassisParamConfigCallback, this));

    //TODO: move the service in slam model
    slam_model_client_ = module_nh_->serviceClient<ginger_msgs::SetSlamModel>("SetSlamModel");

    p_map_ = std::make_shared<ginger::GingerMap>(map_nh_, tf2_buffer_, rdm_pub_);
    if(p_map_->loadDefaultMap()) {
        ros::Duration(1.0).sleep(); // sleep for a second
        p_map_->publishMapAll();
        ros::Duration(1.0).sleep(); // sleep for a second
        p_map_->publishInitialPose();
        cur_mode_ = MODE_NAVIGATION;
        ROS_INFO("NaviModule: load map and set initial pose successfully");
    }
    else {
        ROS_ERROR("GingerNavi: failed to load map");
    }
    p_navi_ = std::make_shared<ginger::GingerNavi>(navi_nh_, p_map_, harix_max_linear_vel_, harix_max_rot_vel_);
    p_navi_->onDockChargingStart([this]() {
        resetRobotPose();
    });
    if (p_navi_)
        p_navi_->ControllDepthScanNode(true); // 导航模式打开depth to scan算法

    // p_navi_->initVel(d_gearSpeed[i_gearSpeed_curr - 1],
    //                 d_turnSpeed[i_turnSpeed_curr - 1]);
*/
    ROS_INFO("NaviModule Initialization Done");
}

NaviModule::~NaviModule()
{
    /*
    if(p_map_) {
        p_map_.reset();
        p_map_ = nullptr;
    }
    if(p_navi_) {
        p_navi_.reset();
        p_navi_ = nullptr;
    }
    */
}

bool NaviModule::startMappingService(ginger_msgs::StartMappingRequest &req, ginger_msgs::StartMappingResponse &res)
{
    ROS_INFO("NaviModule: start calling mapping service");
/*    if(cur_mode_ == MODE_MAPPING) {
        res.result = ginger_msgs::StartMappingResponse::REPEAT_CMD;
        res.description = "Mapping Has Been Started, Don't Repeat";
        ROS_WARN("NaviModule: %s", res.description.c_str());
        return true;
    }
    
    // 清除现在的地图数据
    if (p_map_) {
        p_map_->clearAllMapData();
    }
    
    ginger_msgs::SetSlamModel mapping_srv;
    mapping_srv.request.SlamMode = ginger_msgs::SetSlamModelRequest::SLAM_MAPPING;
    bool flag = slam_model_client_.call(mapping_srv);
    res.result = mapping_srv.response.result;
    res.description = mapping_srv.response.description;
    if (flag) {
        ROS_INFO("NaviModule: start mapping model successfully");
        cur_mode_ = MODE_MAPPING;
        if (p_navi_) {
            p_navi_->ControllDepthScanNode(false); // 建图模式关闭depth to scan算法
        }
    }
    else {
        ROS_ERROR("NaviModule: Failed to start mapping model");
    }
    return flag;
 */   
    return true;
}

/*
bool NaviModule::saveMapService(ginger_msgs::SaveMapRequest &req, ginger_msgs::SaveMapResponse &res)
{
    if(cur_mode_ != MODE_MAPPING) {
        res.result = ginger_msgs::SaveMapResponse::MAPPING_NOT_START;
        res.description = "Cannot save map, because mapping not start";
        ROS_ERROR("NaviModule: %s", res.description.c_str());
        return false;
    }

    geometry_msgs::TransformStamped tfstamed;
    try {
        tfstamed = tf2_buffer_->lookupTransform("map", "base_link", ros::Time(0), ros::Duration(0.1));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
    }
    geometry_msgs::PoseWithCovarianceStamped cur_pose;
    cur_pose.header.frame_id = "map";
    cur_pose.header.stamp = ros::Time::now();
    cur_pose.pose.pose.position.x = tfstamed.transform.translation.x;
    cur_pose.pose.pose.position.y = tfstamed.transform.translation.y;
    cur_pose.pose.pose.orientation = tfstamed.transform.rotation;
    cur_pose.pose.covariance[6*0+0] = 0.5 * 0.5;
    cur_pose.pose.covariance[6*1+1] = 0.5 * 0.5;
    cur_pose.pose.covariance[6*5+5] = (M_PI/12.0) * (M_PI/12.0);

    //save map
    int err = p_map_->saveMap(req.name);
    ROS_INFO("state of saveMap() : %d\n", err);

    if(err > 0) {
        //切换为定位模式
        ginger_msgs::SetSlamModel loc_srv;
        loc_srv.request.SlamMode = ginger_msgs::SetSlamModelRequest::SLAM_LOC;
        bool loc_flag = slam_model_client_.call(loc_srv);
        if (loc_flag) {
            ros::Duration(0.2).sleep(); // sleep
            p_map_->publishMapAll();
            p_map_->publishInitialPose(cur_pose, 1);
            if (p_navi_) {
                p_navi_->ControllDepthScanNode(true); // 导航模式打开depth to scan算法
            }        
            res.result = err;
            res.description = std::string("save map success");
            cur_mode_ = MODE_NAVIGATION_TEM;
            ROS_INFO("NaviModule: save map successfully");
            return true;
        }
    }
    else {
        ROS_ERROR("NaviModule: save map failed, remain mapping mode");
        res.result = err;
        res.description = std::string("save map failed");
        return false;
    }
}

bool NaviModule::loadMapService(ginger_msgs::LoadMapRequest &req, ginger_msgs::LoadMapResponse &res) {
    ROS_INFO("NaviModule: Start calling loadmap service");
    if(cur_mode_ == MODE_MAPPING) {
        res.result = ginger_msgs::LoadMapResponse::MODE_ERROR;
        res.description = "Mapping mode, SaveMap first";
        ROS_ERROR("NaviModule: %s", res.description.c_str());
        return true;
    }

    int err = loadMapImpl(req.name);

    if(err > 0) {
        res.result = ginger_msgs::LoadMapResponse::SUCCESS;
        res.description = "LoadMap Success";
    }
    else if (err == -3) {
        res.result = ginger_msgs::LoadMapResponse::NO_MAP_PNG;
        res.description = "LoadMap Failed, No map png";
    }
    else if (err == -4) {
        res.result = ginger_msgs::LoadMapResponse::NO_MAP_YAML;
        res.description = "LoadMap Failed, No map yaml";
    }
    else if (err == -5) {
        res.result = ginger_msgs::LoadMapResponse::MAP_PNG_UNVALID;
        res.description = "LoadMap Failed, map png invalid";
    }
    else if (err == -6) {
        res.result = ginger_msgs::LoadMapResponse::MAP_YAML_UNVALID;
        res.description = "LoadMap Failed, map yaml invalid";
    }
    else {
        res.result = ginger_msgs::LoadMapResponse::FAILED;
        res.description = "LoadMap Failed";
    }
    return true;
}

int NaviModule::loadMapImpl(std::string map_name) {
    if(p_navi_ != nullptr) {
        // p_navi_.reset();
        // p_navi_ = std::make_shared<ginger::GingerNavi>(navi_nh_, p_map_, harix_max_linear_vel_, harix_max_rot_vel_);
        // 如果正处于任务状态，则先取消任务，再切换地图
        p_navi_->RestNaviState();
    }
    else {
        ROS_ERROR("NaviModule: p_navi_ is null");
        return -1;
    }

    // VecWorldPoints liftPoi;
    std::unordered_map<std::string, LiftPoi> liftPoi;
    int err = p_map_->loadMap(map_name);
    if (err > 0) {
        cur_mode_ = MODE_NAVIGATION;
        //TODO: 如果没有电梯点的情况
        p_map_->getLiftWorldPoint(liftPoi);
        p_navi_->LiftPoiUpdate(liftPoi);
        
        p_map_->publishMapAll();
        //重新加载地图之后，设置充点电为初始位姿
        if (!p_map_->publishInitialPoseChargePoi()) {
            //如果没有获取到充点电，设置(0,0)作为初始点
            p_map_->publishInitialPoseZero();
        }

        // if (p_navi_)
        // {
        //     p_navi_->ControllDepthScanNode(true); // 导航模式打开depth to scan算法
        // }
        // p_navi_->onDockChargingStart([this]() {
        //     resetRobotPose();
        // });
        if(p_navi_->GetNaviCurGoalType() == 1){
            p_navi_->SetPoseInLiftFlag();
        }
    }
    return err;
}

bool NaviModule::checkPoseService(ginger_msgs::CheckPoseRequest &req, ginger_msgs::CheckPoseResponse &res)
{
    if(cur_mode_ != MODE_NAVIGATION) {
        res.result = ginger_msgs::CheckPoseResponse::MODE_ERROR;
        res.description = "Failed to check pose, because not in navi mode";
        ROS_ERROR("NaviModule: %s", res.description.c_str());
        return false;
    }
    ginger_msgs::PixelPose pose;
    bool check_result = p_map_->checkPose(req.name, pose);
    if(check_result) {
        res.pose = pose;
        res.result = ginger_msgs::CheckPoseResponse::SUCCESS;
        res.description = "CheckPose success";
    }
    else {
        res.result = ginger_msgs::CheckPoseResponse::FAILED;
        res.description = "No such pose";
    }

    return check_result;
}

bool NaviModule::setPoseService(ginger_msgs::SetPoseRequest &req, ginger_msgs::SetPoseResponse &res)
{
    if(cur_mode_ != MODE_NAVIGATION) {
        res.result = ginger_msgs::SetPoseResponse::MODE_ERROR;
        res.description = "Failed to set pose, because not in navi mode";
        ROS_ERROR("NaviModule: %s, current model: %d", res.description.c_str(), int(cur_mode_));
        return false;
    }
    geometry_msgs::PoseWithCovarianceStamped w_pose;

    if((p_navi_->GetNaviCurGoalType() == 1) && (p_navi_->GetPoseInLiftFlag())
      && p_map_->isLiftInnerPoi(req.pose)) {
        p_navi_->ClearPoseInLiftFlag();
        if(p_navi_->GetRobotMapPose(cur_lift_in_pose_)){
            w_pose.header.frame_id = "map";
            w_pose.header.stamp = ros::Time::now();
            w_pose.pose.covariance[6*0+0] = 0.5 * 0.5;
            w_pose.pose.covariance[6*1+1] = 0.5 * 0.5;
            w_pose.pose.covariance[6*5+5] = (M_PI/6.0) * (M_PI/6.0);
            w_pose.pose.pose = cur_lift_in_pose_.pose;
            p_map_->publishInitialPose(w_pose, 0);
            res.result = ginger_msgs::SetPoseResponse::SUCCESS;
            res.description = "Lift in state : SetPose Success";
            ROS_INFO("Lift in state : SetPose Success");
        }
        else{
                res.result = ginger_msgs::SetPoseResponse::FAILED;
                res.description = "Lift in state : SetPose  Failed";
                ROS_ERROR("Lift in state : SetPose  Failed");
        }
    }
    else{
        if(p_map_->getWorldPose(req.pose, w_pose.pose.pose)) {
                w_pose.header.frame_id = "map";
                w_pose.header.stamp = ros::Time::now();
                w_pose.pose.covariance[6*0+0] = 0.5 * 0.5;
                w_pose.pose.covariance[6*1+1] = 0.5 * 0.5;
                w_pose.pose.covariance[6*5+5] = (M_PI/6.0) * (M_PI/6.0);
                p_map_->publishInitialPose(w_pose, 1);
                res.result = ginger_msgs::SetPoseResponse::SUCCESS;
                res.description = "SetPose Success";
                ROS_INFO("NaviModule: set initial pose successfully");
            }
            else {
                res.result = ginger_msgs::SetPoseResponse::FAILED;
                res.description = "SetPose Failed";
                ROS_ERROR("NaviModule: failed to set initial pose");
            }
    }
    return true;
}

bool NaviModule::setWorldPoseService(ginger_msgs::SetWorldPoseRequest &req, ginger_msgs::SetWorldPoseResponse &res)
{
    if(cur_mode_ != MODE_NAVIGATION) {
        res.result = ginger_msgs::SetWorldPoseResponse::MODE_ERROR;
        res.description = "Failed to set world pose, because not in navi mode";
        ROS_ERROR("NaviModule: %s, current model: %d", res.description.c_str(), int(cur_mode_));
        return false;
    }

    p_map_->publishInitialPose(req.pose, 0);
    res.result = ginger_msgs::SetWorldPoseResponse::SUCCESS;
    res.description = "SetWorldPose Success";

    return true;
}

bool NaviModule::naviToService(ginger_msgs::NaviToRequest &req, ginger_msgs::NaviToResponse &res)
{
    if (cur_mode_ == MODE_IDLE)
    {
        //最多延时4s
        int i = 20;
        while (navi_nh_->ok() && i-- && cur_mode_ == MODE_IDLE){
            ros::Duration(0.2).sleep();
        }
    }
    // if (cur_mode_ == MODE_NAVIGATION_TEM) {
    //     cur_mode_ = MODE_NAVIGATION;
    // }

    if(cur_mode_ != MODE_NAVIGATION) {
        res.result = ginger_msgs::NaviToResponse::MODE_ERROR;
        res.description = "Not in navigation mode, reject NaviTo Task";
        ROS_ERROR("NaviModule: %s, current model: %d", res.description.c_str(), int(cur_mode_));
        return true;
    }

    // calculate goal pose
    ginger_msgs::PixelPose goal;
    if(req.type == ginger_msgs::NaviToRequest::NORMAL_NAVI) {
        if(!req.name.empty()) {
            ROS_INFO_STREAM("NaviTo task, goal is " << req.name);
            if(!p_map_->checkPose(req.name, goal)) {
                ROS_ERROR("NaviModule: no such poi found");
                res.result = ginger_msgs::NaviToResponse::NAME_UNVALID;
                res.description = "Cant find poi";
                return true;
            }
            else if(req.pose.x > 0.1) {
                float diff_x = std::abs(goal.x - req.pose.x);
                float diff_y = std::abs(goal.y - req.pose.y);
                float diff_a = std::abs(goal.theta - req.pose.theta);
                if(diff_x * diff_y * diff_a > 0.1) {
                    ROS_WARN("NaviModule: receive name and coordinate , but not the same pose");
                    ROS_WARN("NaviModule: (%.0f, %.0f, %.0f) -- (%.0f, %.0f, %.0f)", \
                              goal.x, goal.y, goal.theta, req.pose.x, req.pose.y, req.pose.theta);
                    ROS_WARN("NaviModule: navigation by name");
                }
            }
        }
        else {
            ROS_INFO("NaviTo task, goal at (%.0f, %.0f, %.0f)", req.pose.x, req.pose.y, req.pose.theta);
            goal = req.pose;
        }
    }
    else if(req.type == ginger_msgs::NaviToRequest::LIFT_NAVI) {
        if(!req.name.empty()) {
            ROS_INFO_STREAM("NaviToLift task, goal is " << req.name);
            if(!p_map_->checkPose(req.name, goal)) {
                ROS_ERROR("NaviModule: no such poi found");
                res.result = ginger_msgs::NaviToResponse::NAME_UNVALID;
                res.description = "Cant find poi";
                return true;
            }
        }
        else {
            std::unordered_map<std::string, LiftPoi> liftPoi;
            p_map_->getLiftWorldPoint(liftPoi);
            p_navi_->LiftPoiUpdate(liftPoi);
            goal = req.pose;
            ROS_INFO("NaviToLift task, goal at (%.0f, %.0f, %.0f)", req.pose.x, req.pose.y, req.pose.theta);
        }
    }
    else if(req.type == ginger_msgs::NaviToRequest::LIFT_NAVI_OUT) {
        if(!req.name.empty()) {
            ROS_INFO_STREAM("NaviOutLift task, goal is " << req.name);
            if(!p_map_->checkPose(req.name, goal)) {
                ROS_ERROR("NaviModule: no such poi found");
                res.result = ginger_msgs::NaviToResponse::NAME_UNVALID;
                res.description = "Cant find poi";
                return true;
            }
        }
        else {
            ROS_INFO("NaviOutLift task, goal at (%.0f, %.0f, %.0f)", req.pose.x, req.pose.y, req.pose.theta);
            goal = req.pose;
        }
        goal.theta = ((int)goal.theta + 180) % 360;
    }
    else if(req.type == ginger_msgs::NaviToRequest::CHARGE_NAVI) {
        ginger_msgs::PixelPose charge_pile;
        if(!req.name.empty()) {
            ROS_INFO_STREAM("NaviToCharger task, pile is " << req.name);
            if(!p_map_->queryChargePile(req.name, charge_pile)) {
                ROS_ERROR("charger pile name can not found");
                res.result = ginger_msgs::NaviToResponse::NAME_UNVALID;
                res.description = "Name Invalid";
                return true;
            }
        }
        else if(req.pose.x + req.pose.y + req.pose.theta > 0.1) {
            ROS_INFO("NaviToCharger task, pile at (%.0f, %.0f, %.0f)", req.pose.x, req.pose.y, req.pose.theta);
            charge_pile = req.pose;
        }
        else {
            ROS_INFO("NaviToCharger task, go to nearest pile");
            if(!p_map_->getNearestChargePile(charge_pile)) {
                ROS_ERROR("No charge pile found in the map");
                res.result = ginger_msgs::NaviToResponse::NO_CHARGER_FOUND;
                res.description = "No Charger Found";
                return true;
            }
            else {
                ROS_INFO("NaviModule: find a pile at (%.0f, %.0f, %.0f)", charge_pile.x, charge_pile.y, charge_pile.theta);
            }
        }
        double dock_distance = 0.5;
        goal.x = charge_pile.x + (dock_distance / 0.05) * std::sin(charge_pile.theta * 3.14 / 180.0);
        goal.y = charge_pile.y - (dock_distance / 0.05) * std::cos(charge_pile.theta * 3.14 / 180.0);
        goal.theta = ((int)charge_pile.theta + 180) % 360;
    }
    else {
        ROS_ERROR("NaviModule: unknown navi type received");
        res.result = ginger_msgs::NaviToResponse::FAILED;
        res.description = "Unknown type";
        return true;
    }

    // check goal valid
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.stamp = ros::Time::now();
    goal_pose.header.frame_id = "map";
    if(!p_map_->getWorldPose(goal, goal_pose.pose)) {
        ROS_ERROR("NaviModule: error when map pixel to world");
        res.result = ginger_msgs::NaviToResponse::POSE_UNVALID;
        res.description = "Can't get world pose";
        return true;
    }

    // try to plan
    bool valid = p_map_->isGoalValid(goal);
    // bool valid = p_map_->CheckGoalValid(goal);
    if(!valid) {
        ROS_ERROR("NaviModule: cant get a valid path");
        res.result = ginger_msgs::NaviToResponse::PLAN_FAILED;
        res.description = "plan failed";

        ginger_msgs::RdmAlarm rdm;
        rdm.deviceName = "Navigation";
        rdm.alarmCode = ginger_msgs::RdmAlarm::ALM_NAVI_PATH_FAIL;
        rdm.alarmType.push_back(ginger_msgs::RdmAlarm::ALM_PROCESSING_ERROR);
        rdm.alarmSeverity.push_back(ginger_msgs::RdmAlarm::ALM_CRITICAL);
        rdm_pub_->publish(rdm);

        return true;
    }

    // execute navito task
    if(p_navi_->naviTo(req.task_id, req.type, goal_pose, res)) {
        res.result = ginger_msgs::NaviToResponse::SUCCESS;
        res.description = "Task Accept";
        ROS_INFO("NaviModule: task accepted, go to ** %s ** now", req.name.c_str());
    }

    return true;
}

bool NaviModule::naviCmdService(ginger_msgs::NaviCmdRequest &req, ginger_msgs::NaviCmdResponse &res)
{
    if(cur_mode_ != MODE_NAVIGATION) {
        res.result = ginger_msgs::NaviCmdResponse::MODE_ERROR;
        res.description = "Failed to cmd navi, because not in navi mode";
        ROS_ERROR("NaviModule: %s, current model: %d", res.description.c_str(), int(cur_mode_));
        return false;
    }

    p_navi_->naviCtrl(req, res);
    return true;
}

void NaviModule::resetRobotPose()
{
    if(cur_mode_ != MODE_NAVIGATION) {
        ROS_ERROR("NaviModule: Robot is not in navition model, so cannot reset pose in charge poi: %d", int(cur_mode_));
        return;
    }

    if(p_map_ == nullptr) {
        ROS_ERROR("NaviModule: p_map_ is not initialized, cannot reset robot pose");
        return;
    }

    int initial_flag = 1;
    if (p_map_->publishInitialPoseChargePoi()) {
        ROS_INFO("NaviModule: reset robot pose to charge pile");
    }
}

bool NaviModule::naviVelService(ginger_msgs::NaviVelRequest &req, ginger_msgs::NaviVelResponse &res)
{
    if(cur_mode_ != MODE_NAVIGATION) {
        res.result = false;
        res.description = "Failed to set navi velocity, because not in navi mode";
        ROS_ERROR("NaviModule: %s", res.description.c_str());
        return false;
    }

    ROS_INFO("NaviModule, config speed [%f, %f]", req.linear_max, req.angular_max);
    p_navi_->setVel(req.linear_max, req.angular_max);
    res.result = true;
    res.description = "set success";
    return true;
}

bool NaviModule::navi_vel_set() {
//   double linear_max = 0.0;
//   double angular_max = 0.0;
  if (i_gearSpeed_curr >= 1 && i_gearSpeed_curr <= 3) {
    // linear_max = d_gearSpeed[i_gearSpeed_curr - 1];
    harix_max_linear_vel_ = d_gearSpeed[i_gearSpeed_curr - 1];
  } else {
    ROS_ERROR("navi_vel_set: i_gearSpeed_curr %d is abnormal",
              i_gearSpeed_curr);
    // linear_max = d_gearSpeed[1];
    if (harix_max_linear_vel_ <=0) harix_max_linear_vel_ = d_gearSpeed[1];
  }

  if (i_turnSpeed_curr >= 1 && i_turnSpeed_curr <= 3) {
    // angular_max = d_turnSpeed[i_turnSpeed_curr - 1];
    harix_max_rot_vel_ = d_turnSpeed[i_turnSpeed_curr - 1];
  } else {
    ROS_ERROR("navi_vel_set: i_turnSpeed_curr %d is abnormal",
              i_turnSpeed_curr);
    // angular_max = d_turnSpeed[1];
    if (harix_max_rot_vel_ <= 0) harix_max_rot_vel_ = d_turnSpeed[1];
  }

  if (cur_mode_ != MODE_NAVIGATION) {
    ROS_ERROR("navi_vel_set: failed, Not in Navi mode");
    return false;
  } else {
    p_navi_->setVel(harix_max_linear_vel_, harix_max_rot_vel_);
    ROS_INFO("navi_vel_set: success, harix_max_linear_vel_ %.4f, harix_max_rot_vel_ %.4f",
             harix_max_linear_vel_, harix_max_rot_vel_);
    return true;
  }
}

int NaviModule::ParamUpdate() {
  std::ifstream fin("/home/ginger/ginlt_param/robot_config_hari.yaml");
  try {
    // load yaml file
    YAML::Node speed = YAML::Load(fin);
    if (!speed["moveSpeedHari"]) {
      ROS_ERROR("ParamUpdate: moveSpeedHari item is not exist");
      return -1;
    } else {
      speed["moveSpeedHari"]["gearSpeedCurr"] = i_gearSpeed_curr;
      speed["moveSpeedHari"]["turnSpeedCurr"] = i_turnSpeed_curr;
      std::ofstream paramfile(
          "/home/ginger/ginlt_param/robot_config_hari.yaml");
      paramfile << speed;
      paramfile.close();
    }
  } catch (const YAML::Exception &e) {
    ROS_ERROR("ParamUpdate: parsing robot_config_hari yaml file error!\n %s",
              e.what());
    return -1;
  }
  ROS_INFO("ParamUpdate: update success");
  return 0;
}

bool NaviModule::ChassisParamConfigCallback(
    ginger_msgs::GrpcCommonResp::Request &req,
    ginger_msgs::GrpcCommonResp::Response &res) {
  std::string msgId = req.msgId.c_str();
  Json::Reader reader;
  Json::Value val_json;
  int speed_tmp;

  ROS_INFO("ChassisParamConfigCallback: %s\n, %s", msgId.c_str(),
           req.jsonData.c_str());
  res.result = false;
  if (msgId.compare("2001") == 0) {  // setting
    if (reader.parse(req.jsonData.c_str(), val_json)) {
      if (val_json.isMember("param")) {
        if (val_json["param"].isMember("gearSpeed")) {
          speed_tmp = val_json["param"]["gearSpeed"].asInt();
          if (speed_tmp >= 1 && speed_tmp <= 3) {
            i_gearSpeed_curr = speed_tmp;
            param_nh_->setParam("/moveSpeedHari/gearSpeedCurr",
                                i_gearSpeed_curr);
            res.result = true;
          } else {
            ROS_ERROR("ChassisParamConfigCallback: gearSpeed %d is abnormal",
                      speed_tmp);
          }
        } else if (val_json["param"].isMember("turnSpeed")) {
          speed_tmp = val_json["param"]["turnSpeed"].asInt();
          if (speed_tmp >= 1 && speed_tmp <= 3) {
            i_turnSpeed_curr = speed_tmp;
            param_nh_->setParam("/moveSpeedHari/turnSpeedCurr",
                                i_turnSpeed_curr);
            res.result = true;
          } else {
            ROS_ERROR("ChassisParamConfigCallback: turnSpeed %d is abnormal",
                      speed_tmp);
          }
        } else {
          ROS_ERROR(
              "ChassisParamConfigCallback: \"gearSpeed or turnSpeed\" is not "
              "exist");
        }
      } else {
        ROS_ERROR("ChassisParamConfigCallback: \"param\" is not exist");
      }
    } else {
      ROS_ERROR("ChassisParamConfigCallback: json parse error");
    }
    if (res.result) {
      navi_vel_set();
      ParamUpdate();
      res.jsonData = "{\"code\":0, \"msg\":\"success\"}";
    } else {
      res.jsonData = "{\"code\":-1, \"msg\":\"failed\"}";
    }
  } else if (msgId.compare("2002") == 0) {  // query
    Json::Value rtn_json;
    rtn_json["gearSpeed"] = Json::Value(i_gearSpeed_curr);
    rtn_json["turnSpeed"] = Json::Value(i_turnSpeed_curr);
    res.result = true;
    Json::FastWriter jsonFw;
    res.jsonData = jsonFw.write(rtn_json).c_str();
  }
  ROS_INFO("ChassisParamConfigCallback: gearSpeedCurr %d, turnSpeedCurr %d",
           i_gearSpeed_curr, i_turnSpeed_curr);
  ROS_INFO("ChassisParamConfigCallback: %s", res.jsonData.c_str());

  return true;
}
*/
} // end_ns
