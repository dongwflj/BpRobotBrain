/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/
#include <stdexcept>
#include <ros/ros.h>

#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "robot_ctrl.h"
#include "bt/robot_bt.h"
#include "bt/robot_bt_node.h"

using namespace BT; 

namespace slamengine
{

static const char* xml_text = R"(

    <root main_tree_to_execute = "MainTree" >

        <BehaviorTree ID="MainTree">
			<Fallback name="robot_main_state_check">
				<Sequence name="init_bt">
					<CheckInit name="init_ok"/>
					<Action_InitRobot name="init_robot"/>
				</Sequence>
				<Sequence name="idle_bt">
					<CheckIdle name="idle_ok"/>
                    <SubTree ID="idle_state_tree" robot_event_response="robot_event_response"/>
				</Sequence>
				<Sequence name="mapping_bt">
					<CheckMapping name="mapping_ok"/>
                    <SubTree ID="mapping_state_tree" robot_event_response="robot_event_response"/>
				</Sequence>
				<Sequence name="motioning_bt">
					<CheckMotioning name="motioning_ok"/>
                    <SubTree ID="motioning_state_tree" robot_event_response="robot_event_response"/>
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

        <BehaviorTree ID="idle_state_tree">
            <Fallback name="check_event_fallback">
                <Sequence>
                    <CheckEventStartNavi name="check_event_start_navi_ok"/>
                    <Action_StartNavi name="action_start_navi"/>
                </Sequence>
                <Sequence>
                    <CheckEventStartBuildMap name="check_event_start_build_map_ok"/>
                    <Action_StartBuildMap name="action_start_build_map"/>
                </Sequence>
                <Action_ReturnResponse name="action_return_response" inport_return_response="-2"/>
            </Fallback>
        </BehaviorTree>

        <BehaviorTree ID="mapping_state_tree">
            <Fallback name="check_event_fallback">
                <Sequence>
                    <CheckEventStartBuildMap name="check_event_start_build_map_ok"/>
                    <Action_ReturnResponse name="action_return_response" inport_return_response="-202"/>
                </Sequence>
                <Sequence>
                    <CheckEventStopBuildMap name="check_event_stop_build_map_ok"/>
                    <Action_StopBuildMap name="action_stop_build_map"/>
                </Sequence>
                <Action_ReturnResponse name="action_return_response" inport_return_response="-2"/>
            </Fallback>
        </BehaviorTree>

        <BehaviorTree ID="motioning_state_tree">
            <Fallback name="check_event_fallback">
                <Sequence>
                    <CheckEventStopNavi name="check_event_stop_navi_ok"/>
                    <Action_StopNavi name="action_stop_navi"/>
                </Sequence>
                <Action_ReturnResponse name="action_return_response" inport_return_response="-2"/>
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
    // State Check
    factory.registerSimpleCondition("CheckInit", std::bind(&RobotBt::CheckInit, this));
    factory.registerSimpleCondition("CheckIdle", std::bind(&RobotBt::CheckIdle, this));
	factory.registerSimpleCondition("CheckMapping", std::bind(&RobotBt::CheckMapping, this));
	factory.registerSimpleCondition("CheckMotioning", std::bind(&RobotBt::CheckMotioning, this));
	factory.registerSimpleCondition("CheckStop", std::bind(&RobotBt::CheckStop, this));
	factory.registerSimpleCondition("CheckTeleOping", std::bind(&RobotBt::CheckTeleOping, this));
	factory.registerSimpleCondition("CheckError", std::bind(&RobotBt::CheckError, this));
    // Event Check
	factory.registerSimpleCondition("CheckEventStartBuildMap", std::bind(&RobotBt::CheckEventStartBuildMap, this));
	factory.registerSimpleCondition("CheckEventStopBuildMap", std::bind(&RobotBt::CheckEventStopBuildMap, this));
	factory.registerSimpleCondition("CheckEventStartNavi", std::bind(&RobotBt::CheckEventStartNavi, this));
	factory.registerSimpleCondition("CheckEventStopNavi", std::bind(&RobotBt::CheckEventStopNavi, this));
    // Action
	factory.registerSimpleAction("Action_InitRobot", std::bind(&RobotBt::Action_InitRobot, this));
	factory.registerSimpleAction("Action_StartBuildMap", std::bind(&RobotBt::Action_StartBuildMap, this));
	factory.registerSimpleAction("Action_StopBuildMap", std::bind(&RobotBt::Action_StopBuildMap, this));
	factory.registerSimpleAction("Action_StartNavi", std::bind(&RobotBt::Action_StartNavi, this));
	factory.registerSimpleAction("Action_StopNavi", std::bind(&RobotBt::Action_StopNavi, this));
    // Action node
    factory.registerNodeType<Action_ReturnResponse>("Action_ReturnResponse");

    tree_ = factory.createTreeFromText(xml_text);
    blackboard_ = tree_.rootBlackboard();
    StdCoutLogger logger_cout(tree_);
    printTreeRecursively( tree_.rootNode() );

    blackboard_->set<EROBOTSTATE>("robot_state", INIT);
    tree_.tickRoot();
    ROS_INFO("RobotBt init exit");
}

ERESULT RobotBt::StartBuildMap(const std::string& task_id) {
    ROS_INFO("RobotBt::startBuildMap entry");
    blackboard_->set<EROBOTEVENT>("robot_event", START_BUILD_MAP);
    blackboard_->set<std::string>("robot_event_params", task_id);
    tree_.tickRoot();
    ERESULT res = blackboard_->get<ERESULT>("robot_event_response");
    ROS_INFO("RobotBt::startBuildMap result:%d, exit", res);
    return res;
}

ERESULT RobotBt::StopBuildMap(const std::string& task_id) {
    ROS_INFO("RobotBt::StopBuildMap entry");
    blackboard_->set<EROBOTEVENT>("robot_event", STOP_BUILD_MAP);
    blackboard_->set<std::string>("robot_event_params", task_id);
    tree_.tickRoot();
    ERESULT res = blackboard_->get<ERESULT>("robot_event_response");
	ROS_INFO("RobotBt::StopBuildMap result:%d, exit", res);
    return res;
}

ERESULT RobotBt::StartNavi(const std::string& task_id, ENAVITYPE type, const std::string& goal_name, const PixelPose& goal_pose) {
    ROS_INFO("RobotBt::StartNavi entry");
    blackboard_->set<EROBOTEVENT>("robot_event", START_NAVI);
    EventParams params(task_id, type, goal_name, goal_pose);
    blackboard_->set<EventParams>("robot_event_params", params);
    tree_.tickRoot();
    ERESULT res = blackboard_->get<ERESULT>("robot_event_response");
	ROS_INFO("RobotBt::StartNavi result:%d, exit", res);
    return res;
}

ERESULT RobotBt::StopNavi(const std::string& task_id) {
    ROS_INFO("RobotBt::StopNavi entry");
    blackboard_->set<EROBOTEVENT>("robot_event", STOP_NAVI);
    blackboard_->set<std::string>("robot_event_params", task_id);
    tree_.tickRoot();
    ERESULT res = blackboard_->get<ERESULT>("robot_event_response");
	ROS_INFO("RobotBt::StopNavi result:%d, exit", res);
    return res;
}

ERESULT RobotBt::OnInitDone() {
    ROS_INFO("RobotBt::onInitDone entry");
    blackboard_->set<EROBOTSTATE>("robot_state", IDLE);
    ROS_INFO("RobotBt::onInitDone Robot change to IDLE state exit");
	return E_OK;
}

ERESULT RobotBt::OnNaviDone(const std::string& task_id, int32 state, const std::string& description) {
    ROS_INFO("RobotBt::onNaviDone entry");
    ///blackboard_->set<EROBOTSTATE>("robot_state", IDLE);
    ROS_INFO("RobotBt::onNaviDone Robot change to IDLE state exit");
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
	ROS_INFO("RobotBt::CheckInit state=%d, exit", robot_state);
	return robot_state == INIT ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

BT::NodeStatus RobotBt::CheckIdle() {
    ROS_INFO("RobotBt::CheckIdle entry");
    EROBOTSTATE robot_state = blackboard_->get<EROBOTSTATE>("robot_state");
    ROS_INFO("RobotBt::CheckIdle exit");
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

BT::NodeStatus RobotBt::CheckEventStartBuildMap() {
    ROS_INFO("RobotBt::CheckEventStartBuildMap entry");
    EROBOTEVENT robot_event = blackboard_->get<EROBOTEVENT>("robot_event");
    ROS_INFO("RobotBt::CheckEventStartBuildMap event=%d, event", robot_event);
    return robot_event == START_BUILD_MAP ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

BT::NodeStatus RobotBt::CheckEventStopBuildMap() {
    ROS_INFO("RobotBt::CheckEventStopBuildMap entry");
    EROBOTEVENT robot_event = blackboard_->get<EROBOTEVENT>("robot_event");
    ROS_INFO("RobotBt::CheckEventStopBuildMap event=%d, event", robot_event);
    return robot_event == STOP_BUILD_MAP ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

BT::NodeStatus RobotBt::CheckEventStartNavi() {
    ROS_INFO("RobotBt::CheckEventStartNavi entry");
    EROBOTEVENT robot_event = blackboard_->get<EROBOTEVENT>("robot_event");
    ROS_INFO("RobotBt::CheckEventStartNavi event=%d, event", robot_event);
    return robot_event == START_NAVI ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

BT::NodeStatus RobotBt::CheckEventStopNavi() {
    ROS_INFO("RobotBt::CheckEventStopNavi entry");
    EROBOTEVENT robot_event = blackboard_->get<EROBOTEVENT>("robot_event");
    ROS_INFO("RobotBt::CheckEventStopNavi event=%d, event", robot_event);
    return robot_event == STOP_NAVI ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

BT::NodeStatus RobotBt::Action_InitRobot() {
	ROS_INFO("RobotBt::ActionInitRobot entry");
    robotCtrl_->Init();
	ROS_INFO("RobotBt::ActionInitRobot exit");
	return NodeStatus::SUCCESS;
}

BT::NodeStatus RobotBt::Action_StartBuildMap() {
	ROS_INFO("RobotBt::Action_StartBuildMap entry");
    std::string task_id = blackboard_->get<std::string>("robot_event_params");
    ERESULT res = robotCtrl_->StartBuildMap(task_id);
    blackboard_->set<ERESULT>("robot_event_response", res);
	if (res == E_OK) {
		blackboard_->set<EROBOTSTATE>("robot_state", MAPPING);
		ROS_INFO("Robot StartBuildMap OK, change to MAPPING state");
	}
	else {
		ROS_INFO("Robot StartBuildMap fail:%d", res);
	}
	ROS_INFO("RobotBt::Action_StartBuildMap exit");
	return NodeStatus::SUCCESS;
}

BT::NodeStatus RobotBt::Action_StopBuildMap() {
	ROS_INFO("RobotBt::Action_StopBuildMap entry");
    std::string task_id = blackboard_->get<std::string>("robot_event_params");
    ERESULT res = robotCtrl_->StopBuildMap(task_id);
    blackboard_->set<ERESULT>("robot_event_response", res);
	if (res == E_OK) {
		blackboard_->set<EROBOTSTATE>("robot_state", IDLE);
		ROS_INFO("Robot StopBuildMap OK, change to IDLE state");
	}
	else {
		ROS_INFO("Robot StopBuildMap fail:%d", res);
	}
	ROS_INFO("RobotBt::Action_StopBuildMap exit");
	return NodeStatus::SUCCESS;
}

BT::NodeStatus RobotBt::Action_StartNavi() {
	ROS_INFO("RobotBt::Action_StartNavi entry");
    EventParams params = blackboard_->get<EventParams>("robot_event_params");

    ERESULT res = robotCtrl_->StartNavi(params.task_id_, params.type_,
                                        params.goal_name_, params.goal_pose_);
    blackboard_->set<ERESULT>("robot_event_response", res);
	if (res == E_OK) {
		blackboard_->set<EROBOTSTATE>("robot_state", MOTIONING);
		ROS_INFO("Robot StartNavi OK, change to MOTIONING state");
	}
	else {
		ROS_INFO("Robot StartNavi fail:%d", res);
	}
	ROS_INFO("RobotBt::Action_StartNavi exit");
	return NodeStatus::SUCCESS;
}

BT::NodeStatus RobotBt::Action_StopNavi() {
	ROS_INFO("RobotBt::Action_StopNavi entry");
    std::string task_id = blackboard_->get<std::string>("robot_event_params");
    ERESULT res = robotCtrl_->StopNavi(task_id);
    blackboard_->set<ERESULT>("robot_event_response", res);
	if (res == E_OK) {
		blackboard_->set<EROBOTSTATE>("robot_state", IDLE);
		ROS_INFO("Robot StopNavi OK, change to IDLE state");
	}
	else {
		ROS_INFO("Robot StopNavi fail:%d", res);
	}
	ROS_INFO("RobotBt::Action_StopNavi exit");
	return NodeStatus::SUCCESS;
}


} // end_ns

