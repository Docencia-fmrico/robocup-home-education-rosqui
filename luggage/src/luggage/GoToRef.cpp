// Copyright 2022 ROSqui
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>

#include "luggage/GoToRef.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <move_base_msgs/MoveBaseActionResult.h>

#include "ros/ros.h"

namespace luggage
{

GoToRef::GoToRef(const std::string& name)
: BT::ActionNodeBase(name, {}),
  nh_()
{
	ROS_INFO("CONSTRUCTOR GoToRef");
	result_sub_ = nh_.subscribe("/move_base/result", 1, &GoToRef::ResultCallback, this);
	result_ = 0;
	MyNode my_node_;
  	do_work_ = true;
}

void
GoToRef::halt()
{
  ROS_INFO("GoToRef halt");
}

void
GoToRef::ResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
	result_ = msg->status.status;
}

BT::NodeStatus
GoToRef::tick()
{
	if(do_work_){
		my_node_.doWork(200, coords_);
		do_work_ = false;
	}

	ROS_INFO("Result: %d", result_);
	//std::cerr << "LOG: result: " << result_ << std::endl;

	if (result_ == 3)
	{
		ROS_INFO("LEAVING");
		return BT::NodeStatus::SUCCESS;
	}

  	return BT::NodeStatus::RUNNING;
}
}  // namespace luggage

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<luggage::GoToRef>("GoToRef");
}
