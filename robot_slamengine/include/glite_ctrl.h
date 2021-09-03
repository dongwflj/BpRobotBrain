/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/
#ifndef _GLITE_CTRL_H_
#define _GLITE_CTRL_H_

#include "slamengine/robot_ctrl.h"

namespace slamengine {
class IRobotObserver;
}

namespace ginger
{
using namespace slamengine;
class GliteCtrl : public IRobotCtrl
{
public:
    ERESULT startBuildMap();
    ERESULT stopBuildMap();
    ERESULT pauseBuildMap();
    ERESULT resumeBuildMap();
    ERESULT saveMap();
    ERESULT loadMap();  

    ERESULT setObserver(IRobotObserver&);
private:
    IRobotObserver *observer_;
};

}

#endif
