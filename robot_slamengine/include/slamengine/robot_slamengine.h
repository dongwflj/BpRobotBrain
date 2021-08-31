/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_SLAMENGINE_H
#define _ROBOT_SLAMENGINE_H

namespace slamengine
{

class RobotSlamEngine {
public:
    static RobotSlamEngine& getInstance() {
        static RobotSlamEngine instance;
        return instance;
    }
private:
    RobotSlamEngine();
    ~RobotSlamEngine() {};
    RobotSlamEngine(const RobotSlamEngine&) {};
    RobotSlamEngine& operator=(const RobotSlamEngine&) {};
};

} // end_ns

#endif
