/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_CTRL_H_
#define _ROBOT_CTRL_H_

#include "define.h"

namespace slamengine
{

class IRobotObserver;

class IRobotCtrl {
public:
    virtual ERESULT startBuildMap() = 0;
    virtual ERESULT stopBuildMap() = 0;
    virtual ERESULT pauseBuildMap() = 0;
    virtual ERESULT resumeBuildMap() = 0;
    virtual ERESULT saveMap() = 0;
    virtual ERESULT loadMap() = 0;  

    virtual ERESULT startNavi(ENAVITYPE type) = 0;
    virtual ERESULT stopNavi() = 0;
    virtual ERESULT setObserver(IRobotObserver&) = 0;
};

} // end_ns

#endif
