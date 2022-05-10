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

#ifndef LUGGAGE_START_H
#define LUGGAGE_START_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <cv_bridge/cv_bridge.h>

#include "luggage/Dialog.h"

#include <string>
#include <vector>
#include "ros/ros.h"

namespace luggage
{

class Start : public BT::ActionNodeBase
{
  public:
    explicit Start(const std::string& name);
    void halt();
    BT::NodeStatus tick();

  private:
    ros::NodeHandle nh_;
    bool first_;
    luggage::Dialog forwarder_;
};

}  // namespace luggage

#endif  // LUGGAGE_START_H
