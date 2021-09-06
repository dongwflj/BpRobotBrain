/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_SLAMENGINE_H_
#define _ROBOT_SLAMENGINE_H_

#include "define.h"

namespace slamengine
{
class IRobotCtrl;
class IRobotEngineObserver;
class RobotFsm;

class RobotSlamEngine {
public:
    static RobotSlamEngine& getInstance() {
        static RobotSlamEngine instance;
        return instance;
    }
    // Must call before any func call once you get instance
    void init(IRobotCtrl& robotCtrl, IRobotEngineObserver& robotEngineObserver);
public:
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
	
    ERESULT move(){};
    ERESULT setPose(){};
    ERESULT setWorldPose(){};
    ERESULT setRobotVelThreshold(){};
	//Event
    ERESULT notifyDropEvent(){};
    ERESULT notifyCollisionEvent(){};
    ERESULT notifyEStopEvent(){};
    ERESULT notifyDockStateEvent(){};
    ERESULT notifyChargingEvent(){};
    ERESULT notifyCriticalHwErrEvent(){};
private:
    RobotSlamEngine();
    virtual ~RobotSlamEngine();
    RobotSlamEngine(const RobotSlamEngine&) {};
    RobotSlamEngine& operator=(const RobotSlamEngine&) {};
    RobotFsm* robotFsm_;
};

} // end_ns

#endif
