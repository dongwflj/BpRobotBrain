/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/
#include <stdexcept>
#include <ros/ros.h>

#include "robot_ctrl.h"
#include "bt/robot_bt.h"
#include "bt/robot_start_build_map.h"

using namespace BT; 

namespace slamengine
{

static const char* xml_text = R"(

    <root main_tree_to_execute = "MainTree" >

        <BehaviorTree ID="MainTree">
			<Fallback name="robot_main">
				<Sequence name="init_bt">
					<CheckInit name="init_ok"/>
				</Sequence>
				<Sequence name="idle_bt">
					<CheckIdle name="idle_ok"/>
				</Sequence>
				<Sequence name="mapping_bt">
					<CheckMapping name="mapping_ok"/>
				</Sequence>
				<Sequence name="motioning_bt">
					<CheckMotioning name="motioning_ok"/>
				</Sequence>
				<Sequence name="stop_bt">
					<CheckStop name="stop_ok"/>
				</Sequence>
				<Sequence name="teleoping_bt">
					<CheckTeleOping name="teleoping_ok"/>
				</Sequence>
				<Sequence name="error_bt">
					<CheckError name="error_ok"/>
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
    robotCtrl_->SetObserver(*this);

	BehaviorTreeFactory factory;

    factory.registerNodeType<RobotStartBuildMap>("RobotStartBuildMap");
    factory.registerSimpleCondition("CheckInit", std::bind(&RobotBt::CheckInit, this));
    factory.registerSimpleCondition("CheckIdle", std::bind(&RobotBt::CheckIdle, this));
	factory.registerSimpleCondition("CheckMapping", std::bind(&RobotBt::CheckMapping, this));
	factory.registerSimpleCondition("CheckMotining", std::bind(&RobotBt::CheckMotioning, this));
	factory.registerSimpleCondition("CheckStop", std::bind(&RobotBt::CheckStop, this));
	factory.registerSimpleCondition("CheckTeleOping", std::bind(&RobotBt::CheckTeleOping, this));
	factory.registerSimpleCondition("CheckError", std::bind(&RobotBt::CheckError, this));

    blackboard_ = Blackboard::create();
    tree_ = factory.createTreeFromText(xml_text, blackboard_);

    blackboard_->set<EROBOTSTATE>("robot_state", INIT);
    robotCtrl_->Init();
    ROS_INFO("RobotBt init exit");
}

ERESULT RobotBt::TriggerEvent(const EROBOTEVENT event, const std::string& task_id) {
    ROS_INFO("RobotBt::TriggerEvent entry");
    blackboard_->set("para", task_id);
    tree_.tickRoot();
    std::string res = blackboard_->get<std::string>("response");
    std::cout << res <<std::endl;
    ROS_INFO("RobotBt::TriggerEvent exit");
    return E_OK;
}

ERESULT RobotBt::StartBuildMap(const std::string& task_id) {
    ROS_INFO("RobotFsm::startBuildMap entry");
    ROS_INFO("RobotFsm::startBuildMap exit");
    return E_OK;
}

ERESULT RobotBt::OnInitDone() {
    ROS_INFO("RobotBt::onInitDone entry");
    blackboard_->set<EROBOTSTATE>("robot_state", IDLE);
    ROS_INFO("RobotBt::onInitDone exit");
	return E_OK;
}

ERESULT RobotBt::OnNaviDone(const std::string& task_id, int32 state, const std::string& description) {
    ROS_INFO("RobotBt::onNaviDone entry");
    ROS_INFO("RobotBt::onNaviDone exit");
    return E_OK;
}

ERESULT RobotBt::OnNaviActive(const std::string& task_id, bool active) {
    ROS_INFO("RobotBt::onNaviActive entry");
    return E_OK;
}

ERESULT RobotBt::OnNaviProgress(const std::string& task_id, const Pose& curr_pose) {
    ROS_INFO("RobotBt::onNaviProgress entry");
    return E_OK;
}

BT::NodeStatus RobotBt::CheckInit() {
	ROS_INFO("RobotBt::CheckInit entry");
	EROBOTSTATE robot_state = blackboard_->get<EROBOTSTATE>("robot_state");
	return robot_state == INIT ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
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

BT::NodeStatus RobotBt::CheckTeleOping() {
	ROS_INFO("RobotBt::CheckTeleOping entry");
	EROBOTSTATE robot_state = blackboard_->get<EROBOTSTATE>("robot_state");
	return robot_state == TELEOPING ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

BT::NodeStatus RobotBt::CheckError() {
	ROS_INFO("RobotBt::CheckError entry");
	EROBOTSTATE robot_state = blackboard_->get<EROBOTSTATE>("robot_state");
	return robot_state == ERROR ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

BT::NodeStatus RobotBt::ActionInitRobot() {
	ROS_INFO("RobotBt::ActionInitRobot entry");
	ROS_INFO("RobotBt::ActionInitRobot exit");
	return NodeStatus::SUCCESS;
}

} // end_ns

