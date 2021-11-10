/*
* Copyright (c) 2021, BytePowers, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_MAPPING_STATE_H_
#define _ROBOT_MAPPING_STATE_H_

#include "robot_state.h"
#include <string>

namespace slamengine
{

class RobotMappingState : public RobotState {
public:
    static RobotState& GetInstance() {
        static RobotMappingState instance;
        return instance;
    }
 
    ERESULT StartBuildMap(const std::string& task_id);
    ERESULT StopBuildMap(const std::string& task_id);
    ERESULT PauseBuildMap(const std::string& task_id);
    ERESULT ResumeBuildMap(const std::string& task_id);
    ERESULT SaveMap(const std::string& task_id, const std::string& map_name);
    ERESULT LoadMap(const std::string& task_id, const std::string& map_name);
private:
    RobotMappingState() {};
    virtual ~RobotMappingState() {};
    RobotMappingState(const RobotMappingState&) {};
    RobotMappingState& operator=(const RobotMappingState&) {};
};

} // end_ns

#endif
