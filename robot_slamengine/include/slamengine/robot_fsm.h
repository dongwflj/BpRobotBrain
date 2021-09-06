/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_FSM_H_
#define _ROBOT_FSM_H_

#include "slamengine/robot_observer.h"

namespace slamengine
{
class RobotState;
class IRobotCtrl;
class IRobotEngineObserver;

class RobotFsm : public IRobotObserver {
public:
    virtual ~RobotFsm();
    ERESULT startBuildMap();
    ERESULT stopBuildMap();
    ERESULT pauseBuildMap();
    ERESULT resumeBuildMap();
    ERESULT saveMap();
    ERESULT loadMap(); 
	// Navi
    ERESULT startNavi(ENAVITYPE type);
    ERESULT stopNavi();
    ERESULT pauseNavi();
    ERESULT resumeNavi();
	
    ERESULT setPose(){};
    ERESULT setWorldPose(){};
    ERESULT setRobotVelThreshold(){};
    ERESULT move(){};
	//Event
    ERESULT notifyDropEvent(){};
    ERESULT notifyCollisionEvent(){};
    ERESULT notifyEStopEvent(){};
    ERESULT notifyDockStateEvent(){};
    ERESULT notifyChargingEvent(){};
    ERESULT notifyCriticalHwErrEvent(){};
    //IRobotObserver
    ERESULT onNaviDone();
    ERESULT onNaviActive();
    ERESULT onNaviProgress();
    //IRobotFsm 
    void transitionTo(RobotState &state); 
    IRobotCtrl& getRobotCtrl(); 
    IRobotEngineObserver& getRobotEngineObserver(); 
public:
    // Must call before any func call once you get instance
    void init(IRobotCtrl& robotCtrl, IRobotEngineObserver& robotEngineObserver);
private:
    IRobotCtrl* robotCtrl_;
    IRobotEngineObserver* robotEngineObserver_;

    RobotState* state_;
};

} // end_ns
#endif
