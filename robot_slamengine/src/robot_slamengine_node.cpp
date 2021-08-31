/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/

#include <ros/ros.h>

#include "NaviModule.h"

int main(int argc, char** argv)
{ 
    ros::init(argc, argv, "robot_slamengine_nonde");
    ros::start();
    ginger::NaviModule engine;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
