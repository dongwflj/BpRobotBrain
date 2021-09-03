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
class IRobotFsm;
class RobotState {
public:
    virtual ~RobotState() {}
    void set_context(IRobotFsm *context) {
        this->context_ = context;
    }

    virtual ERESULT startBuildMap();
    virtual ERESULT stopBuildMap();
    virtual ERESULT pauseBuildMap();
    virtual ERESULT resumeBuildMap();
    
	virtual ERESULT onNaviDone();
    virtual ERESULT onNaviActive();
    virtual ERESULT onNaviProgress();
 
protected:
    IRobotFsm *context_;
};

} // end_ns

#endif
