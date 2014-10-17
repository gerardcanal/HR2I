#ifndef _ROS_text_to_speech_SoundAction_h
#define _ROS_text_to_speech_SoundAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "text_to_speech/SoundActionGoal.h"
#include "text_to_speech/SoundActionResult.h"
#include "text_to_speech/SoundActionFeedback.h"

namespace text_to_speech
{

  class SoundAction : public ros::Msg
  {
    public:
      text_to_speech::SoundActionGoal action_goal;
      text_to_speech::SoundActionResult action_result;
      text_to_speech::SoundActionFeedback action_feedback;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "text_to_speech/SoundAction"; };
    const char * getMD5(){ return "14c227d084c36b42c2f833bb4ab427d0"; };

  };

}
#endif