/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_MAPPING_STATE_H_
#define _ROBOT_MAPPING_STATE_H_

#include "robot_state.h"
#include <string>
using namespace std;

namespace slamengine
{

class RobotMappingState : public RobotState {
public:
    static RobotState& getInstance() {
        static RobotMappingState instance;
        return instance;
    }
 
    ERESULT StartBuildMap(string task_id);
    ERESULT StopBuildMap(string task_id);
    ERESULT PauseBuildMap(string task_id);
    ERESULT ResumeBuildMap(string task_id);
private:
    RobotMappingState() {};
    virtual ~RobotMappingState() {};
    RobotMappingState(const RobotMappingState&) {};
    RobotMappingState& operator=(const RobotMappingState&) {};
};

} // end_ns

#endif
