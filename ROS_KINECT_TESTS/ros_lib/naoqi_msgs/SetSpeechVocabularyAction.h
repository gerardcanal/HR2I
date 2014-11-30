#ifndef _ROS_naoqi_msgs_SetSpeechVocabularyAction_h
#define _ROS_naoqi_msgs_SetSpeechVocabularyAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "naoqi_msgs/SetSpeechVocabularyActionGoal.h"
#include "naoqi_msgs/SetSpeechVocabularyActionResult.h"
#include "naoqi_msgs/SetSpeechVocabularyActionFeedback.h"

namespace naoqi_msgs
{

  class SetSpeechVocabularyAction : public ros::Msg
  {
    public:
      naoqi_msgs::SetSpeechVocabularyActionGoal action_goal;
      naoqi_msgs::SetSpeechVocabularyActionResult action_result;
      naoqi_msgs::SetSpeechVocabularyActionFeedback action_feedback;

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

    const char * getType(){ return "naoqi_msgs/SetSpeechVocabularyAction"; };
    const char * getMD5(){ return "737441a71b3375ccf5219f84239ade13"; };

  };

}
#endif