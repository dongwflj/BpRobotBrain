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
class RobotFsm;
class RobotSlamEngine {
public:
    static RobotSlamEngine& getInstance() {
        static RobotSlamEngine instance;
        return instance;
    }
    ERESULT startBuildMap();
    ERESULT stopBuildMap();
    ERESULT pauseBuildMap() {};
    ERESULT resumeBuildMap() {};
    ERESULT saveMap();
    ERESULT loadMap();  

private:
    RobotSlamEngine();
    ~RobotSlamEngine() {};
    RobotSlamEngine(const RobotSlamEngine&) {};
    RobotSlamEngine& operator=(const RobotSlamEngine&) {};
    RobotFsm *fsm_;
};

} // end_ns

#endif
