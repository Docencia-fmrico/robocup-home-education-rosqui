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

#include "luggage/PercievePerson.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace luggage
{

PercievePerson::PercievePerson(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config),
  nh_(),
  image_depth_sub(nh_, "/camera/depth/image_raw", 1),
  image_color_sub(nh_, "/camera/rgb/image_raw", 1),
  bbx_sub(nh_, "/darknet_ros/bounding_boxes", 1),
  sync_bbx(MySyncPolicy_bbx(10), image_depth_sub, bbx_sub)
{
  detected = false;
  first = true;
  sync_bbx.registerCallback(boost::bind(&PercievePerson::callback_bbx, this, _1, _2));
}

void PercievePerson::callback_bbx(const sensor_msgs::ImageConstPtr& image,
const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
{
  cv_bridge::CvImagePtr img_ptr;
  try
  {
      img_ptr = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::TYPE_32FC3);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception:  %s", e.what());
      return;
  }

  detected = false;

  int px = 0;
  int py = 0;
  float dist = 0;

  int length_x = (max_x - min_x);
  int length_y = (max_y - min_y);

  int half_y = length_y / 2;

  int last_pixel_y = max_y - length_y/6;

  int red = 0;
  int green = 0;
  int blue = 0;
  
  int prev_red = 0;
  int prev_green = 0;
  int prev_blue = 0;

  float prob = 0;
  // Darknet only detects person
  for (const auto & box : boxes->bounding_boxes)
  {
    ROS_INFO("PROB: %f", box.probability);
    if (box.probability > 0.65)
    {
      min_x = box.xmin;
      max_x = box.xmax;
      min_y = box.ymin;
      max_y = box.ymax;
      ///////////////// GET B.BOX R,G,B ////////////////
      for (int x = min_x; x < max_x; x++)
      {
        for (int y = half_y; y <  last_pixel_y; y++)
        {
          red = img_ptr->image.at<cv::Vec3b>(y, x)[0] + red;
          green = img_ptr->image.at<cv::Vec3b>(y, x)[1] + green;
          blue = img_ptr->image.at<cv::Vec3b>(y, x)[2] + blue;
        }
        red = red / (last_pixel_y - half_y);
        green = green / (last_pixel_y - half_y);
        blue = blue / (last_pixel_y - half_y);
      }

      red = red / length_x;
      green = green / length_x;
      blue = blue / length_x;
      ////////////////////////////////////////////////

      // If current box's color is closer to color_ than the previous box's.
      if (compare_colors(red, green, blue, prev_red, prev_green, prev_blue))
      {
        ROS_INFO("DETECTED TRUE");

        prev_red = red;
        prev_green = green;
        prev_blue = blue;

        detected = true;
        px = (box.xmax + box.xmin) / 2;
        py = (box.ymax + box.ymin) / 2;

        dist = img_ptr->image.at<float>(cv::Point(px, py))*0.001f;
        if (isnan(dist))
          dist = 0;

        ROS_INFO("person_x: %d \t person_z: %f\n", px, dist);
      }
    }
  }
  // ESTOS OUTPUTS PUEDEN DAR PROBLEMAS AQUI.
  setOutput("person_x", px);
  setOutput("person_z", dist);
}

bool
PercievePerson::compare_colors(int R1, int G1, int B1, int R2, int G2, int B2)
{
  // Return value:
  // True if RGB1 is closer to color_ than RGB2
  // False otherwise


  // Set colors as coordinates and calculates distances.
  bool closer = false;

  int R0 = color_[0];
  int G0 = color_[1];
  int B0 = color_[2];

  float D1 = sqrt((R1-R0)^2+(G1-G0)^2+(B1-B0)^2);
  float D2 = sqrt((R2-R0)^2+(G2-G0)^2+(B2-B0)^2);

  if (D1 < D2)
  {
    closer = true;
  }
  return closer;
}

void
PercievePerson::halt()
{
  ROS_INFO("PercievePerson halt");
}

BT::NodeStatus
PercievePerson::tick()
{
  if (first)
  {
    color_ = getInput<std::vector<int>>("color").value();
    initial_ts_ = ros::Time::now();
    first = false;
  }
  
  double current_ts_ = (ros::Time::now() - initial_ts_).toSec();

  ROS_INFO("current_ts_: %f", current_ts_);

  if ( detected )
  {
    // Jumps to FollowPerson
    first = true;
    detected = false;
    return BT::NodeStatus::SUCCESS;
  }
  else if ((!detected) && (current_ts_ > 2.0))
  {
    // Jumps to turn
    // ROS_INFO("Detected: FALSE");
    return BT::NodeStatus::FAILURE;
  }
  else
  {
    return BT::NodeStatus::RUNNING;
  }
}
}  // namespace luggage

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<luggage::PercievePerson>("PercievePerson");
}
