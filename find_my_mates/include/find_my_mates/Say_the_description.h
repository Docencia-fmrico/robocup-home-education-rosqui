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
/*
#ifndef SAYTHEDESCRIPTION__H
#define SAYTHEDESCRIPTION__H

#include <find_my_mates/DialogInterface.h>
#include <sound_play/SoundRequest.h>
#include <string>

namespace ph = std::placeholders;

namespace find_my_mates
{
class Say_the_description: public DialogInterface
{
  public:
    Say_the_description(): nh_()
    {
      this->registerCallback(std::bind(&Say_the_description::noIntentCB, this, ph::_1));
      this->registerCallback(
        std::bind(&Say_the_description::DetectL, this, ph::_1),
        "Detect Luggage");
    }

    void noIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[Say_the_description] noIntentCB: intent [%s]", result.intent.c_str());
      listen();
    }

    void DetectL(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[Say_the_description] introduceIntentCB: intent [%s]", result.intent.c_str());
      
      for (const auto & param : result.parameters) {
        std::cerr << param << std::endl;
        for (const auto & value : param.value) {
          std::cerr << "\t" << value << std::endl;
        }
      }
      speak(result.fulfillment_text);
    }

  private:
    ros::NodeHandle nh_;
};
};  // namespace find_my_mates

#endif
*/