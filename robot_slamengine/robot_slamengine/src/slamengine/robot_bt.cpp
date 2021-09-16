/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/
#include <stdexcept>
#include <ros/ros.h>

#include "slamengine/robot_bt.h"
#include "slamengine/robot_ctrl.h"
#include "slamengine/robot_engine_observer.h"
#include "slamengine/robot_start_build_map.h"

using namespace BT; 

namespace slamengine
{

static const char* xml_text = R"(

    <root main_tree_to_execute = "MainTree" >

        <BehaviorTree ID="MainTree">
            <Fallback name="robot_main">
                <Sequence name="mapping_bt">
                    <CheckMapping name="mapping_ok"/>
                    <RobotStartBuildMap  para="${para}"/>
                </Sequence>
                <Sequence name="idle_bt">
                    <CheckIdle name="idle_ok"/>
                    <RobotStartBuildMap  para="${para}"/>
                </Sequence>
            </Fallback>
        </BehaviorTree>

    </root>
    )";

RobotBt::~RobotBt() {
    ROS_INFO("RobotBt entry");
    ROS_INFO("RobotBt exit");
}

void RobotBt::Init(IRobotCtrl& robotCtrl, IRobotEngineObserver& robotEngineObserver) {
    ROS_INFO("RobotBt init entry");
    robotCtrl_ = &robotCtrl;
    robotEngineObserver_ = &robotEngineObserver;
    robotCtrl_->setObserver(*this);

	BehaviorTreeFactory factory;

    factory.registerNodeType<RobotStartBuildMap>("RobotStartBuildMap");
    factory.registerSimpleCondition("CheckIdle", std::bind(&RobotBt::CheckIdle, this));
    factory.registerSimpleCondition("CheckMapping", std::bind(&RobotBt::CheckMapping, this));
    blackboard_ = Blackboard::create();
    tree_ = factory.createTreeFromText(xml_text, blackboard_);

    blackboard_->set("robot_state", IDLE);
    // Simulate trigger build map
    TriggerEvent(START_BUILD_MAP, "aaa");
    ///tree_.tickRoot();
    ROS_INFO("RobotBt init exit");
}

ERESULT RobotBt::TriggerEvent(EVENT event, string task_id) {
    ROS_INFO("RobotBt::TriggerEvent entry");
    blackboard_->set("para", task_id);
    tree_.tickRoot();
    string res = blackboard_->get<string>("response");
    std::cout << res <<std::endl;
    ROS_INFO("RobotBt::TriggerEvent exit");
    return E_OK;
}

ERESULT RobotBt::StartBuildMap(string task_id) {
    ROS_INFO("RobotFsm::startBuildMap entry");
    ROS_INFO("RobotFsm::startBuildMap exit");
    return E_OK;
}


ERESULT RobotBt::onNaviDone(string task_id, int32 state, string description) {
    ROS_INFO("RobotBt::onNaviDone entry");
    ROS_INFO("RobotBt::onNaviDone exit");
    return E_OK;
}

ERESULT RobotBt::onNaviActive(string task_id, bool active) {
    ROS_INFO("RobotBt::onNaviActive entry");
    return E_OK;
}

ERESULT RobotBt::onNaviProgress(string task_id, Pose curr_pose) {
    ROS_INFO("RobotBt::onNaviProgress entry");
    return E_OK;
}

BT::NodeStatus RobotBt::CheckIdle() {
    ROS_INFO("RobotBt::CheckIdle entry");
    EROBOTSTATE robot_state = blackboard_->get<EROBOTSTATE>("robot_state");
    return robot_state == IDLE ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

BT::NodeStatus RobotBt::CheckMapping() {
    ROS_INFO("RobotBt::CheckMapping entry");
    EROBOTSTATE robot_state = blackboard_->get<EROBOTSTATE>("robot_state");
    return robot_state == MAPPING ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

BT::NodeStatus RobotBt::CheckMotioning() {
    ROS_INFO("RobotBt::CheckMotioning entry");
    EROBOTSTATE robot_state = blackboard_->get<EROBOTSTATE>("robot_state");
    return robot_state == MOTIONING ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

BT::NodeStatus RobotBt::CheckStop() {
    ROS_INFO("RobotBt::CheckStop entry");
    EROBOTSTATE robot_state = blackboard_->get<EROBOTSTATE>("robot_state");
    return robot_state == STOP ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

} // end_ns

