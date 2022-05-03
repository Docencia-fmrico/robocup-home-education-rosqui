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
    explicit AnalyzePerson(): nh_()
    {
      this->registerCallback(std::bind(&AnalyzePerson::noIntentCB, this, ph::_1));
      this->registerCallback(
        std::bind(&AnalyzePerson::DetectLCB, this, ph::_1),
        "Detect Luggage");
      this->registerCallback(
        std::bind(&AnalyzePerson::PresentationCB, this, ph::_1),
        "Presentation");
      
    }

    void noIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[Dialog] noIntentCB: intent [%s]", result.intent.c_str());
      ros::Duration(1, 0).sleep();
      speak("Sorry, can you repeat it please?");
      listen();
    }

    void PresentationCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[Dialog] PresentationCB: intent [%s]", result.intent.c_str());
      
      for (const auto & param : result.parameters) {
        std::cerr << param << std::endl;
        for (const auto & value : param.value) {
          std::cerr << "\t" << value << std::endl;
        }
      } 
      speak(result.fulfillment_text);
      listen();
    }

    void DetectLCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[Dialog] DetectLCB: intent [%s]", result.intent.c_str());
      
      for (const auto & param : result.parameters) {
        std::cerr << param << std::endl;
        for (const auto & value : param.value) {
          std::cerr << "\t" << value << std::endl;
        }
      }
      speak(result.fulfillment_text);
      listen();
    }
    void halt();

    BT::NodeStatus tick();

  private:
    ros::NodeHandle nh_;
};

}  // namespace find_my_mates

#endif  // FIND_MY_MATES_AnalyzePerson_H
