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

#ifndef FIND_MY_MATES_AnalyzePerson_H
#define FIND_MY_MATES_AnalyzePerson_H

#include <find_my_mates/DialogInterface.h>
#include <sound_play/SoundRequest.h>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"


namespace ph = std::placeholders;

namespace find_my_mates
{
class AnalyzePerson : public DialogInterface
{
  public:
    explicit AnalyzePerson(const std::string& name, const BT::NodeConfiguration & config);
    void callback_bbx(const sensor_msgs::ImageConstPtr& image,
    const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<std::string>("bag_pos"), BT::OutputPort<std::vector<int>>("color")};
    }

    void halt();

    BT::NodeStatus tick();
    
  private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::Image> image_color_sub;
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
    darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
    message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx;
    int min_x;
    int max_x;
    std::vector<int> color_ = {0,0,0};

};

}  // namespace find_my_mates

#endif  // FIND_MY_MATES_AnalyzePerson_H
