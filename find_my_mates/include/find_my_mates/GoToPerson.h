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

#ifndef FINDMYMATES_GOTOPERSON_H
#define FINDMYMATES_GOTOPERSON_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <message_filters/subscriber.h>

#include <move_base_msgs/MoveBaseActionResult.h>

#include "geometry_msgs/Twist.h"

#include <string>
#include <vector>

namespace find_my_mates
{

class GoToPerson : public BT::ActionNodeBase
{
  public:
    explicit GoToPerson(const std::string& name);
    void halt();
    BT::NodeStatus tick();
    void ResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber result_sub_;
    int result_;
    bool first_;

    int current_pos_;

    std::vector<float> coords1 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    std::vector<float> coords2 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    std::vector<float> coords3 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    std::vector<float> coords4 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    std::vector<float> coords5 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    std::vector<float> coords6 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

    std::vector<std::vector<float>> all_coords = {coords1, coords2, coords3, coords4, coords5, coords6};
    
};

}  // namespace find_my_mates

#endif  // FINDMYMATES_GOTOPERSON_H
