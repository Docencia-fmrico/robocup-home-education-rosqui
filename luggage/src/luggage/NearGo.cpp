// Copyright 2022 Intelligent Robotics Lab
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

#include "luggage/NearGo.h"

#include "geometry_msgs/Twist.h"

#include "ros/ros.h"

namespace luggage
{

// Constructor
NearGo::NearGo()
{
  detected_ = false;
  state_ = GOING_FORWARD;
  // n_ es el NodeHandler. Se encarga de suscribir y publicar donde haga falta.
  sub_laser_scan_ = n_.subscribe("/scan_filtered", 1, &NearGo::scanFilteredCallback, this);
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
}

float
NearGo::average_range(const sensor_msgs::LaserScan::ConstPtr& msg, int sector,
                      int ranges_in_sector, int front, int num_ranges)
{
  int num_valid_rangs = ranges_in_sector;
  float average;
  float sum_valid_rangs = 0;

  if (front != 0 || sector >= 0)
  {
    for (int i = front + (sector*ranges_in_sector); i < front + ((sector+1)*ranges_in_sector); i++)
    {
      if (msg->ranges[i] > msg->range_max || msg->ranges[i] < msg->range_min)
      {
        num_valid_rangs--;
      }
      else
      {
        sum_valid_rangs += msg->ranges[i];
      }
    }
  }
  else if (sector < 0 && front == 0)
  { // front == 0
    for (int i = num_ranges + (sector*ranges_in_sector); i < num_ranges + ((sector+1)*ranges_in_sector); i++)
    {
      if (msg->ranges[i] > msg->range_max || msg->ranges[i] < msg->range_min)
      {
        num_valid_rangs--;
      }
      else
      {
        sum_valid_rangs += msg->ranges[i];
      }
    }
  }
  if (!num_valid_rangs)
  { //  num_valid_rangs = 0 => no valid lectures in sector, so sector is faced to inf
    average = msg->range_max;
  }
  else
  {
    average = sum_valid_rangs/num_valid_rangs;
  }
  // ROS_INFO("Average sector: %d = %f", sector, average); // Traza para calibrar el RPlidar
  return average;
}

// checks 20 sectors from:
// front -> right (number_ranges * proportion_to_check(0-0.5))
// front -> left
// returns the sector wich average > MIN_RANGE_LASER
// else returns NUM_SECTORS +1
int
NearGo::check_sector(const sensor_msgs::LaserScan::ConstPtr& msg, int num_ranges, float proportion_to_check, int front)
{
  int warning_sector = NUM_SECTORS+1;
  // num_ranges * proportion in right side/ num sector in right side
  // this value is simetric in the left side
  int ranges_in_sector = (num_ranges * proportion_to_check)/NUM_SECTORS;
  float sector_rang_avg;

  for (int sector = -NUM_SECTORS; sector < NUM_SECTORS; sector++)
  {
    if (MIN_RANGE_LASER > average_range(msg, sector, ranges_in_sector, front, num_ranges))
    {
      warning_sector = sector;
      ROS_INFO("WARNING SECTOR : %d", warning_sector);
      break;
    }
  }
  return warning_sector;
}

void
NearGo::scanFilteredCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  float proportion = 0.1875;  // 3/16
  int num_ranges = (msg->angle_max-msg->angle_min)/msg->angle_increment;
  int front = num_ranges/2;  // num_ranges/2 para simulador
  int near_sector = check_sector(msg, num_ranges, proportion, front);

  if (near_sector > -FRONT_LIMIT_SECT && near_sector < FRONT_LIMIT_SECT)
  {
    detected_ = true;
    direction_ = DETECTED_FRONT;
    ROS_INFO("DETECTED_FRONT");
  }
  else
  {
    detected_ = false;
  }
}

bool
NearGo::scan()
{
      if (detected_)
      {
        ROS_INFO("OBSTACLE DETECTED");
        return true;
      }
      else
      {
        ROS_INFO("NO OBSTACLE");
        return false;
      }
}

}  // namespace luggage
