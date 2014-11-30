#ifndef _ROS_naoqi_msgs_SetSpeechVocabularyActionResult_h
#define _ROS_naoqi_msgs_SetSpeechVocabularyActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "naoqi_msgs/SetSpeechVocabularyResult.h"

namespace naoqi_msgs
{

  class SetSpeechVocabularyActionResult : public ros::Msg
  {
    public:
      std_msgs::Header header;
      actionlib_msgs::GoalStatus status;
      naoqi_msgs::SetSpeechVocabularyResult result;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "naoqi_msgs/SetSpeechVocabularyActionResult"; };
    const char * getMD5(){ return "0698ce25b2d595b82357c010557e935f"; };

  };

}
#endif