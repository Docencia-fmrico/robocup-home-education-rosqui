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
        std::bind(&Dialog::DetectL, this, ph::_1),
        "Detect Luggage");
      this->registerCallback(
        std::bind(&Dialog::Presentation, this, ph::_1),
        "Presentation");
      
    }

    void noIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[Dialog] noIntentCB: intent [%s]", result.intent.c_str());
      ros::Duration(1, 0).sleep();
      speak("Sorry, can you repeat it please?");
      listen();
    }

    void Presentation(dialogflow_ros_msgs::DialogflowResult result)
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

    void DetectL(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[Dialog] introduceIntentCB: intent [%s]", result.intent.c_str());
      
      for (const auto & param : result.parameters) {
        std::cerr << param << std::endl;
        for (const auto & value : param.value) {
          std::cerr << "\t" << value << std::endl;
        }
      }
      speak(result.fulfillment_text);
      listen();
    }

  private:
    ros::NodeHandle nh_;
};
}  // namespace luggage

