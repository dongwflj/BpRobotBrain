/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_SLAMENGINE_H_
#define _ROBOT_SLAMENGINE_H_

#include <string>
#include "define.h"

namespace slamengine
{
class IRobotCtrl;
class IRobotEngineObserver;
class RobotFsm;
class RobotBt;

class RobotSlamEngine {
public:
    static RobotSlamEngine& GetInstance() {
        static RobotSlamEngine instance;
        return instance;
    }
    // Must call before any func call once you get instance
    void Init(IRobotCtrl& robotCtrl, IRobotEngineObserver& robotEngineObserver);
public:
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
	
    ERESULT Move(){};
    ERESULT CheckPose(const PixelPose & pixel_pose){};
    ERESULT SetPose(const PixelPose& pixel_pose){};
    ERESULT SetWorldPose(const Pose& world_pose){};
    ERESULT SetRobotVelThreshold(float32 linear_max, float32 angular_max){};

    ERESULT StartHA(const std::string& task_id){};
    ERESULT StopHA(const std::string& task_id){};
    //Event
    ERESULT NotifyDropEvent(DropType type);
    ERESULT NotifyCollisionEvent(CollisionType type);
    ERESULT NotifyEStopEvent(EmergencyType type);
    ERESULT NotifyChargingEvent(const BatteryState& state);
    ERESULT NotifyCriticalHwErrEvent();
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
