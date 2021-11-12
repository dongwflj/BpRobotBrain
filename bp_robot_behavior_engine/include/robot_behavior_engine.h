/*
* Copyright (c) 2021, BytePowers, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_BEHAVIOR_ENGINE_H_
#define _ROBOT_BEHAVIOR_ENGINE_H_

#include <string>
#include "define.h"

namespace bp
{
class IRobotCtrl;
class IRobotEngineObserver;
class RobotBt;

class RobotBehaviorEngine {
public:
    static RobotBehaviorEngine& GetInstance() {
        static RobotBehaviorEngine instance;
        return instance;
    }
    // Must call before any func call once you get instance
    ERESULT Init(IRobotCtrl& robotCtrl, IRobotEngineObserver& robotEngineObserver);
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
    RobotBehaviorEngine();
    virtual ~RobotBehaviorEngine();
    RobotBehaviorEngine(const RobotBehaviorEngine&) {};
    RobotBehaviorEngine& operator=(const RobotBehaviorEngine&) {};
    RobotBt*  robotBt_;
};

} // end_ns

#endif
