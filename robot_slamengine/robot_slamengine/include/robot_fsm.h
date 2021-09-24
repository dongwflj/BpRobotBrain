/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_FSM_H_
#define _ROBOT_FSM_H_

#include <string.h>
#include <stack>

#include "robot_observer.h"

namespace slamengine
{
class RobotState;
class IRobotCtrl;
class IRobotEngineObserver;

class RobotFsm : public IRobotObserver {
public:
    virtual ~RobotFsm();
    ERESULT StartBuildMap(const std::string& task_id);
    ERESULT StopBuildMap(const std::string& task_id);
    ERESULT PauseBuildMap(const std::string& task_id);
    ERESULT ResumeBuildMap(const std::string& task_id);
    ERESULT SaveMap(const std::string& task_id, const std::string& map_name);
    ERESULT LoadMap(const std::string& task_id, const std::string& map_name); 
    // Navi
    ERESULT StartNavi(const std::string& task_id, ENAVITYPE type, const std::string& goal_name, const PixelPose& goal_pose);
    ERESULT StopNavi(const std::string& task_id);
    ERESULT PauseNavi(const std::string& task_id);
    ERESULT ResumeNavi(const std::string& task_id);
	
    ERESULT SetPose(const PixelPose& pixel_pose){};
    ERESULT SetWorldPose(const Pose& world_pose){};
    ERESULT SetRobotVelThreshold(float32 linear_max, float32 angular_max){};
    ERESULT Move(){};
    ERESULT Resume();
    //Event
    ERESULT NotifyDropEvent(DropType type);
    ERESULT NotifyCollisionEvent(CollisionType type);
    ERESULT NotifyEStopEvent(EmergencyType type);
    ERESULT NotifyChargingEvent(const BatteryState& state);
    ERESULT NotifyCriticalHwErrEvent();
    //IRobotObserver
    ERESULT OnInitDone();
    ERESULT OnNaviDone(const std::string& task_id, int32 state, const std::string& description);
    ERESULT OnNaviActive(const std::string& task_id, bool active);
    ERESULT OnNaviProgress(const std::string& task_id, const Pose& curr_pose);
    //IRobotFsm 
    void TransitionTo(RobotState& state); 
    IRobotCtrl& GetRobotCtrl(); 
    IRobotEngineObserver& GetRobotEngineObserver(); 
public:
    // Must call before any func call once you get instance
    void Init(IRobotCtrl& robotCtrl, IRobotEngineObserver& robotEngineObserver);
private:
    IRobotCtrl* robotCtrl_;
    IRobotEngineObserver* robotEngineObserver_;
    RobotState* state_;
};

} // end_ns
#endif
