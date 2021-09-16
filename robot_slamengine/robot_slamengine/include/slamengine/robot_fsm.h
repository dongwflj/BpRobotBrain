/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_FSM_H_
#define _ROBOT_FSM_H_

#include <string.h>
#include "slamengine/robot_observer.h"

using namespace std;

namespace slamengine
{
class RobotState;
class IRobotCtrl;
class IRobotEngineObserver;

class RobotFsm : public IRobotObserver {
public:
    virtual ~RobotFsm();
    ERESULT StartBuildMap(string task_id);
    ERESULT StopBuildMap(string task_id);
    ERESULT PauseBuildMap(string task_id);
    ERESULT ResumeBuildMap(string task_id);
    ERESULT SaveMap(string task_id, string map_name);
    ERESULT LoadMap(string task_id, string map_name); 
	// Navi
    ERESULT StartNavi(string task_id, ENAVITYPE type, string goal_name, PixelPose goal_pose);
    ERESULT StopNavi(string task_id);
    ERESULT PauseNavi(string task_id);
    ERESULT ResumeNavi(string task_id);
	
    ERESULT SetPose(PixelPose pixel_pose){};
    ERESULT SetWorldPose(Pose world_pose){};
    ERESULT SetRobotVelThreshold(float32 linear_max, float32 angular_max){};
    ERESULT Move(){};
    //Event
    ERESULT NotifyDropEvent(DropType type){};
    ERESULT NotifyCollisionEvent(CollisionType type){};
    ERESULT NotifyEStopEvent(bool stop){};
    ERESULT NotifyChargingEvent(BatteryState state){};
    ERESULT NotifyCriticalHwErrEvent(){};
    //IRobotObserver
    ERESULT onNaviDone(string task_id, int32 state, string description);
    ERESULT onNaviActive(string task_id, bool active);
    ERESULT onNaviProgress(string task_id, Pose curr_pose);
    //IRobotFsm 
    void transitionTo(RobotState &state); 
    IRobotCtrl& getRobotCtrl(); 
    IRobotEngineObserver& getRobotEngineObserver(); 
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
