/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_SLAMENGINE_H_
#define _ROBOT_SLAMENGINE_H_

#include "define.h"
#include <string>

using namespace std;

namespace slamengine
{
class IRobotCtrl;
class IRobotEngineObserver;
class RobotFsm;
class RobotBt;

class RobotSlamEngine {
public:
    static RobotSlamEngine& getInstance() {
        static RobotSlamEngine instance;
        return instance;
    }
    // Must call before any func call once you get instance
    void Init(IRobotCtrl& robotCtrl, IRobotEngineObserver& robotEngineObserver);
public:
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
	
    ERESULT Move(){};
    ERESULT SetPose(PixelPose pixel_pose){};
    ERESULT SetWorldPose(Pose world_pose){};
    ERESULT SetRobotVelThreshold(float32 linear_max, float32 angular_max){};

    ERESULT StartHA(string task_id){};
    ERESULT StopHA(string task_id){};
    //Event
    ERESULT NotifyDropEvent(DropType type){};
    ERESULT NotifyCollisionEvent(CollisionType type){};
    ERESULT NotifyEStopEvent(bool stop){};
    ERESULT NotifyChargingEvent(BatteryState state){};
    ERESULT NotifyCriticalHwErrEvent(){};
private:
    RobotSlamEngine();
    virtual ~RobotSlamEngine();
    RobotSlamEngine(const RobotSlamEngine&) {};
    RobotSlamEngine& operator=(const RobotSlamEngine&) {};
    RobotFsm* robotFsm_;
    RobotBt*  robotBt_;
};

} // end_ns

#endif
