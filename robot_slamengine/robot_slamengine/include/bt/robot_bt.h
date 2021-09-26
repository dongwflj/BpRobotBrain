/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef ROBOT_BT_H
#define ROBOT_BT_H

#include <behaviortree_cpp_v3/bt_factory.h>
#include "behaviortree_cpp_v3/behavior_tree.h" 
#include "robot_observer.h"

namespace slamengine
{
class RobotState;
class IRobotCtrl;
class IRobotEngineObserver;


class EventParams {
public:
    EventParams(const std::string& task_id, const ENAVITYPE type, const std::string& goal_name,
                 const PixelPose& goal_pose) : 
                 task_id_(task_id), type_(type), goal_name_(goal_name), goal_pose_(goal_pose) {
    }
    // If we needn't return prev state, we need keep from_state_ is nullptr
    std::string task_id_;
    ENAVITYPE type_;
    std::string goal_name_;
    PixelPose goal_pose_;
};

class RobotBt : public IRobotObserver {
public:
    virtual ~RobotBt();
    // Must call before any func call once you get instance
    void Init(IRobotCtrl& robotCtrl, IRobotEngineObserver& robotEngineObserver);

    ERESULT StartBuildMap(const std::string& task_id);
    ERESULT StopBuildMap(const std::string& task_id);
    ERESULT StartNavi(const std::string& task_id, ENAVITYPE type, const std::string& goal_name, const PixelPose& goal_pose);
    ERESULT StopNavi(const std::string& task_id);

    //IRobotObserver
    ERESULT OnInitDone();
    ERESULT OnNaviDone(const std::string& task_id, int32 state, const std::string& description);
    ERESULT OnNaviActive(const std::string& task_id, bool active);
    ERESULT OnNaviProgress(const std::string& task_id, const Pose& curr_pose);

protected:
    // State Condition
    BT::NodeStatus CheckInit(); 
    BT::NodeStatus CheckIdle(); 
    BT::NodeStatus CheckMapping(); 
    BT::NodeStatus CheckMotioning(); 
    BT::NodeStatus CheckStop(); 
	BT::NodeStatus CheckTeleOping();
	BT::NodeStatus CheckError();
    //Event Condition 
    BT::NodeStatus CheckEventStartBuildMap();
    BT::NodeStatus CheckEventStopBuildMap();
    BT::NodeStatus CheckEventStartNavi();
    BT::NodeStatus CheckEventStopNavi();
    // Action
	BT::NodeStatus Action_InitRobot();
    BT::NodeStatus Action_StartBuildMap(); 
    BT::NodeStatus Action_StopBuildMap(); 
    BT::NodeStatus Action_StartNavi(); 
    BT::NodeStatus Action_StopNavi(); 
private:
    IRobotCtrl* robotCtrl_;
    IRobotEngineObserver* robotEngineObserver_;
    BT::Tree tree_;
    BT::Blackboard::Ptr blackboard_;
};

} // end_ns
#endif
