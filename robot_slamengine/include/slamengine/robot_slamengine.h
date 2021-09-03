/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_SLAMENGINE_H_
#define _ROBOT_SLAMENGINE_H_

#include "define.h"
#include "robot_observer.h"
#include "robot_fsm.h"

namespace slamengine
{
class RobotState;
class IRobotCtrl;

class RobotSlamEngine : public IRobotObserver, IRobotFsm {
public:
    ERESULT startBuildMap();
    ERESULT stopBuildMap();
    ERESULT pauseBuildMap();
    ERESULT resumeBuildMap();
    ERESULT saveMap();
    ERESULT loadMap(); 
	// Navi
    ERESULT StartNavi(){};
    ERESULT StopNavi(){};
    ERESULT PauseNavi(){};
    ERESULT ResumeNavi(){};
	
    ERESULT SetPose(){};
    ERESULT SetWorldPose(){};
    ERESULT SetRobotVelThreshold(){};
    ERESULT Move(){};
	//Event
    ERESULT NotifyDropEvent(){};
    ERESULT NotifyCollisionEvent(){};
    ERESULT NotifyEStopEvent(){};
    ERESULT NotifyDockStateEvent(){};
    ERESULT NotifyChargingEvent(){};
    ERESULT NotifyCriticalHwErrEvent(){};
    //IRobotObserver
    ERESULT onNaviDone();
    ERESULT onNaviActive();
    ERESULT onNaviProgress();
    //IRobotFsm 
    void transitionTo(RobotState *state); 
    IRobotCtrl& getRobotCtrl(); 
    IRobotObserver& getRobotObserver(); 
public:
    static RobotSlamEngine& getInstance() {
        static RobotSlamEngine instance;
        return instance;
    }
    // Must call before any func call once you get instance
    void init(IRobotCtrl& robotCtrl, IRobotObserver& robotObserver);
private:
    RobotSlamEngine();
    virtual ~RobotSlamEngine();
    RobotSlamEngine(const RobotSlamEngine&) {};
    RobotSlamEngine& operator=(const RobotSlamEngine&) {};
    RobotState *state_;
    IRobotCtrl *robotCtrl_;
    IRobotObserver *robotObserver_;
};

} // end_ns

#endif
