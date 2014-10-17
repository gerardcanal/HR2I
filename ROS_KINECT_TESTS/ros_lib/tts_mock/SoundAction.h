#ifndef _ROS_tts_mock_SoundAction_h
#define _ROS_tts_mock_SoundAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "tts_mock/SoundActionGoal.h"
#include "tts_mock/SoundActionResult.h"
#include "tts_mock/SoundActionFeedback.h"

namespace tts_mock
{

  class SoundAction : public ros::Msg
  {
    public:
      tts_mock::SoundActionGoal action_goal;
      tts_mock::SoundActionResult action_result;
      tts_mock::SoundActionFeedback action_feedback;

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

    const char * getType(){ return "tts_mock/SoundAction"; };
    const char * getMD5(){ return "14c227d084c36b42c2f833bb4ab427d0"; };

  };

}
#endif