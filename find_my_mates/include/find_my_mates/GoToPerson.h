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

    std::vector<float> coords1 = {1.24, 6.43, 0.0, 0.0, 0.0, 0.73, 0.67};
    std::vector<float> coords2 = {-0.7, 6.15, 0.0, 0.0, 0.0, 0.84, 0.52};
    std::vector<float> coords3 = {-0.77, 5.82, 0.0, 0.0, 0.0, 0.99, 0.02};
    std::vector<float> coords4 = {-0.15, 4.28, 0.0, 0.0, 0.0, -0.94, 0.32};
    std::vector<float> coords5 = {0.53, 3.44, 0.0, 0.0, 0.0, -0.76, 0.64};
    std::vector<float> coords6 = {2.21, 3.21, 0.0, 0.0, 0.0, -0.69, 0.71};

    std::vector<std::vector<float>> all_coords = {coords1, coords2, coords3, coords4, coords5, coords6};
    
};

}  // namespace find_my_mates

#endif  // FINDMYMATES_GOTOPERSON_H
