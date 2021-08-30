/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

namespace slamengine
{

NaviModule::NaviModule(): cur_mode_(MODE_IDLE)
{
    ROS_INFO("NaviModule initializing...");
    ROS_INFO("NaviModule Initialization Done");
}
} // end_ns

int main(int argc, char** argv)
{ 
    ros::init(argc, argv, "robot_slamengine");
    ros::start();
    slamengin::RobotSlamEngine engin;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
