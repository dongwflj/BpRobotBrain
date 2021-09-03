/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_OBSERVER_H_
#define _ROBOT_OBSERVER_H_

#include "define.h"

namespace slamengine
{

class IRobotObserver {
public:
    virtual ERESULT onNaviDone() = 0;
    virtual ERESULT onNaviActive() = 0;
    virtual ERESULT onNaviProgress() = 0;
};

} // end_ns

#endif
