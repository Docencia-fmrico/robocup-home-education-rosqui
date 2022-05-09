
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

#include "luggage/Lost.h"
#include <string>
#include "luggage/Dialog.h"

namespace luggage
{

Lost::Lost(const std::string& name)
: BT::ActionNodeBase(name, {})
{
    pub_vel_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);
    detected_ts_ = ros::Time::now();
    time_ = 1;
}

void
Lost::halt()
{
  ROS_INFO("Lost halt");
}

BT::NodeStatus
Lost::tick()
{
  ROS_INFO("Lost tick");
  geometry_msgs::Twist cmd;
  luggage::Dialog forwarder_;

  if ((ros::Time::now() - detected_ts_).toSec() > time_)
  {
    detected_ts_ = ros::Time::now();
    forwarder_.speak("I am lost referee");
    time_++;
  }

  cmd.linear.x = 0;
  cmd.angular.z = TURN_VEL;

  pub_vel_.publish(cmd);
  return BT::NodeStatus::FAILURE;
}

}  // namespace luggage

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<luggage::Lost>("Lost");
}
