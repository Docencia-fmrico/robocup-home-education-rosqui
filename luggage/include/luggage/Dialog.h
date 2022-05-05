/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2018, Intelligent Robotics Labs
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Jonatan Gines jginesclavero@gmail.com */

/* Mantainer: Jonatan Gines jginesclavero@gmail.com */

#ifndef LUGGAGE_DIALOG_H
#define LUGGAGE_DIALOG_H

#include <luggage/DialogInterface.h>
#include <sound_play/SoundRequest.h>
#include <string>

namespace ph = std::placeholders;

namespace luggage
{
class Dialog : public DialogInterface
{
  public:
    Dialog(): nh_()
    {
      this->registerCallback(std::bind(&Dialog::noIntentCB, this, ph::_1));
      this->registerCallback(
        std::bind(&Dialog::DetectLCB, this, ph::_1),
        "Detect Luggage");
      this->registerCallback(
        std::bind(&Dialog::PresentationCB, this, ph::_1),
        "Presentation");
      this->registerCallback(
        std::bind(&Dialog::StartCB, this, ph::_1),
        "Start");
    }

    void noIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[Dialog] noIntentCB: intent [%s]", result.intent.c_str());
      speak("Sorry, can you repeat it please?");
      listen();
    }

    void PresentationCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[Dialog] PresentationCB: intent [%s]", result.intent.c_str());

      for (const auto & param : result.parameters)
      {
        std::cerr << param << std::endl;
        for (const auto & value : param.value)
        {
          std::cerr << "\t" << value << std::endl;
        }
      }
      speak(result.fulfillment_text);
    }

    void DetectLCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[Dialog] DetectLCB: intent [%s]", result.intent.c_str());

      side_ = result;

      for (const auto & param : result.parameters)
      {
        std::cerr << param << std::endl;
        for (const auto & value : param.value)
        {
          std::cerr << "\t" << value << std::endl;
        }
      }
      speak(result.fulfillment_text);
    }

    void StartCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[Dialog] StartCB: intent [%s]", result.intent.c_str());
      start_ = 0;
      first_ = 0;
      speak(result.fulfillment_text);
    }

    int get_start()
    {
      ROS_INFO("first_: %d",first_);
      return first_;
    }

    int get_first()
    {
      ROS_INFO("start_: %d",start_);
      return start_;
    }

    dialogflow_ros_msgs::DialogflowResult getValue()
    {
      return side_;
    }

  private:
    ros::NodeHandle nh_;
    dialogflow_ros_msgs::DialogflowResult side_;
    int start_ = 1;
    int first_ = 1;
};
}  // namespace luggage

#endif  // LUGGAGE_DIALOG_H
