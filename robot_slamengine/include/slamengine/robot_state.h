/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_STATE_H_
#define _ROBOT_STATE_H_

#include "define.h"

namespace slamengine
{
class RobotFsm;
class RobotState {
public:
    virtual ~RobotState() {}
    void set_context(RobotFsm *context) {
        this->context_ = context;
    }

    virtual ERESULT startBuildMap();
    virtual ERESULT stopBuildMap();
    virtual ERESULT pauseBuildMap();
    virtual ERESULT resumeBuildMap();
    
	// Navi
    virtual ERESULT startNavi(ENAVITYPE type);
    virtual ERESULT stopNavi();
    virtual ERESULT pauseNavi();
    virtual ERESULT resumeNavi();

    virtual ERESULT naviDoneEvent();
    virtual ERESULT naviActiveEvent();
    virtual ERESULT naviProgressEvent();
 
protected:
    RobotFsm *context_;
};

} // end_ns

#endif
