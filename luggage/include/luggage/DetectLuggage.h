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

#ifndef LUGGAGE_DETECTLUGGAGE_H
#define LUGGAGE_DETECTLUGGAGE_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <string>
#include <vector>

namespace luggage
{

class DetectLuggage : public BT::ActionNodeBase
{
  public:
    explicit DetectLuggage(const std::string& name, const BT::NodeConfiguration & config);
    void callback_bbx(const sensor_msgs::ImageConstPtr& image,
    const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);
    void getPredominantColor(int red, int green, int blue);
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
    int min_y;
    int max_y;
    
    ros::Time listen_t;
    std::vector<int> color_ = {0,0,0};

};

}  // namespace luggage

#endif  // LUGGAGE_DETECTLUGGAGE_H
