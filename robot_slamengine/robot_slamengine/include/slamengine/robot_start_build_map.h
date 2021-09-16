/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef ROBOT_START_BUILD_MAP_H
#define ROBOT_START_BUILD_MAP_H

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/behavior_tree.h"  

using namespace BT;

namespace slamengine
{

class RobotStartBuildMap : public BT::SyncActionNode {
public:
    RobotStartBuildMap(const std::string& name, const NodeConfiguration& config) :
        BT::SyncActionNode(name, config) {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() {
        return { InputPort<std::string>("para") };
    }
};

} // end_ns
#endif
