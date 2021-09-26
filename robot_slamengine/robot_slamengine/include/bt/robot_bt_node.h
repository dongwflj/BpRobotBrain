/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_BT_NODE_H_
#define _ROBOT_BT_NODE_H_

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/behavior_tree.h"  

namespace BT {
template <> inline ERESULT convertFromString(StringView str) {
	printf("Converting string: \"%s\"\n", str.data() );
	ERESULT res;
	res = (ERESULT)convertFromString<int>(str.data());
	return res;
}
}

using namespace BT;

namespace slamengine
{

class Action_ReturnResponse: public BT::SyncActionNode {
public:
    Action_ReturnResponse(const std::string& name, const NodeConfiguration& config) :
        BT::SyncActionNode(name, config) {
    }
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() {
        return { InputPort<ERESULT>("inport_return_response") };
    }
};

} // end_ns
#endif
