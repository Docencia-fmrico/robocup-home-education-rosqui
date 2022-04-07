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

#include "luggage/GoToBag.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace luggage
{

GoToBag::GoToBag(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config),
  nh_()
{
  ROS_INFO("CONSTRUCTOR BAG");
  pub_vel_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
  first = true;

}

void
GoToBag::halt()
{
  ROS_INFO("GoToBag halt");
}

BT::NodeStatus
GoToBag::tick()
{
  if (first)
  {
    detected_ts_ = ros::Time::now();
    bag_pos_ = getInput<std::string>("bag_pos").value();
    first = false;
  }
  
  geometry_msgs::Twist cmd;
  double current_ts_ = (ros::Time::now() - detected_ts_).toSec();
  ROS_INFO("TIME: %f", current_ts_);
  
  if ( (current_ts_ < ACTION_TIME_) ||  ((current_ts_ > 2*ACTION_TIME_) && (current_ts_ < 3*ACTION_TIME_)) )
  {
      cmd.linear.x = FORWARD_VEL;
      cmd.angular.z = 0;
      ROS_INFO("TIME: %f %s", current_ts_, "FORWARD");
      pub_vel_.publish(cmd);
      return BT::NodeStatus::RUNNING;

  }
  else if (current_ts_ <= 2*ACTION_TIME_)
  {
      cmd.linear.x = 0;

      if (bag_pos_ == "right")
        cmd.angular.z = TURNING_VEL_;
      else
        cmd.angular.z = -TURNING_VEL_;

      ROS_INFO("TIME: %f %f", current_ts_, TURNING_VEL_);
      pub_vel_.publish(cmd);
      return BT::NodeStatus::RUNNING;

  }
  else
  {
      ROS_INFO("BAG REACHED");
      return BT::NodeStatus::SUCCESS;
  }

}
}  // namespace luggage

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<luggage::GoToBag>("GoToBag");
}