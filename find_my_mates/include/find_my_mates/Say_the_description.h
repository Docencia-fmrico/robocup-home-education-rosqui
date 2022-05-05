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

#ifndef SAYTHEDESCRIPTION__H
#define SAYTHEDESCRIPTION__H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <cv_bridge/cv_bridge.h>

#include <string>
#include <vector>
#include "ros/ros.h"

namespace ph = std::placeholders;

namespace find_my_mates
{
class Say_the_description: public BT::ActionNodeBase
{
  public:
    explicit Say_the_description(const std::string& name, const BT::NodeConfiguration & config);
    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<std::string>("bag_pos"), BT::OutputPort<std::vector<int>>("color")};
    }

    void halt();
    BT::NodeStatus tick();
    
  private:
    ros::NodeHandle nh_;
};
};  // namespace find_my_mates

#endif
