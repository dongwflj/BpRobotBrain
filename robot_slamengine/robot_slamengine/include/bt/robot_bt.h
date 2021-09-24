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

class RobotBt : public IRobotObserver {
public:
    virtual ~RobotBt();
    // Must call before any func call once you get instance
    void Init(IRobotCtrl& robotCtrl, IRobotEngineObserver& robotEngineObserver);

    ERESULT TriggerEvent(const EROBOTEVENT event, const std::string& task_id);
    ERESULT StartBuildMap(const std::string& task_id);
    //IRobotObserver
    ERESULT OnInitDone();
    ERESULT OnNaviDone(const std::string& task_id, int32 state, const std::string& description);
    ERESULT OnNaviActive(const std::string& task_id, bool active);
    ERESULT OnNaviProgress(const std::string& task_id, const Pose& curr_pose);

protected:
    BT::NodeStatus CheckInit(); 
    BT::NodeStatus CheckIdle(); 
    BT::NodeStatus CheckMapping(); 
    BT::NodeStatus CheckMotioning(); 
    BT::NodeStatus CheckStop(); 
	BT::NodeStatus CheckTeleOping();
	BT::NodeStatus CheckError();
	BT::NodeStatus ActionInitRobot();
private:
    IRobotCtrl* robotCtrl_;
    IRobotEngineObserver* robotEngineObserver_;
    BT::Tree tree_;
    BT::Blackboard::Ptr blackboard_;
};

} // end_ns
#endif
