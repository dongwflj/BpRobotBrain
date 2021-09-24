/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_IDLE_STATE_H_
#define _ROBOT_IDLE_STATE_H_

#include <string>
#include "robot_state.h"

namespace slamengine
{

class RobotIdleState : public RobotState {
public:
    static RobotState& GetInstance() {
        static RobotIdleState instance;
        return instance;
    }
 
    ERESULT StartBuildMap(const std::string& task_id);
    ERESULT StartNavi(const std::string& task_id, ENAVITYPE type, const std::string& goal_name, const PixelPose& goal_pose);
    ERESULT SaveMap(const std::string& task_id, const std::string& map_name);
    ERESULT LoadMap(const std::string& task_id, const std::string& map_name);  
private:
    RobotIdleState() {};
    virtual ~RobotIdleState() {};
    RobotIdleState(const RobotIdleState&) {};
    RobotIdleState& operator=(const RobotIdleState&) {};
};

} // end_ns

#endif
