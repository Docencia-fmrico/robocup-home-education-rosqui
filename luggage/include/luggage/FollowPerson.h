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

#ifndef LUGGAGE_FOLLOWPERSON_H
#define LUGGAGE_FOLLOWPERSON_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "luggage/PIDController.h"

#include <string>
#include <iostream>

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

namespace luggage
{

class FollowPerson : public BT::ActionNodeBase
{
  public:
    explicit FollowPerson(const std::string& name, const BT::NodeConfiguration & config);

    void halt();
    char getch();
    BT::NodeStatus tick();

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<double>("person_z"), BT::InputPort<int>("person_x")};
    }

  protected:
    ros::NodeHandle nh_;
    ros::Publisher pub_vel_;
    PIDController linear_pid_;
    PIDController angular_pid_;
};

}  // namespace luggage

#endif  // LUGGAGE_FOLLOWPERSON_H
