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

#include "luggage/DetectLuggage.h"

#include "luggage/Dialog.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace luggage
{

DetectLuggage::DetectLuggage(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config),
  nh_()
{
  first_ = true;
}
 
void
DetectLuggage::halt()
{
  ROS_INFO("DetectLuggage halt");
}

BT::NodeStatus
DetectLuggage::tick()
{
  ROS_INFO("Detect Luggage Tick");
  if (first_)
  {
    detected_ts_ = ros::Time::now();
    forwarder_.listen();
    //forwarder_.speak("Is luggage at your right or at your left?");
    first_ = false;
  }
    double current_ts_ = (ros::Time::now() - detected_ts_).toSec();
    if ( (current_ts_ > 20))
    {
        ROS_INFO("TIME EXCEEDED");
        setOutput("bag_pos", "right");
        ROS_INFO("OUTPUT SET");
        return BT::NodeStatus::SUCCESS;
    }

    dialogflow_ros_msgs::DialogflowResult side;
    side = forwarder_.getValue();
    for (const auto & param : side.parameters)
    {
      for (const auto & value : param.value)
      {
        if (value == "left")   
        {
          forwarder_.speak("Ok, put the left luggage on me please");
          ROS_INFO("USER'S LEFT");
          setOutput("bag_pos", "left");
          return BT::NodeStatus::SUCCESS;
        }
        else if (value == "right")  
        {
          forwarder_.speak("Ok, put the right luggage on me please");
          ROS_INFO("USER'S RIGHT");
          setOutput("bag_pos", "right");
          return BT::NodeStatus::SUCCESS;
        }
      }
    }
  return BT::NodeStatus::RUNNING;
}
}  // namespace luggage

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<luggage::DetectLuggage>("DetectLuggage");
}
