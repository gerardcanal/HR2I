#ifndef _ROS_naoqi_msgs_SetSpeechVocabularyActionGoal_h
#define _ROS_naoqi_msgs_SetSpeechVocabularyActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "naoqi_msgs/SetSpeechVocabularyGoal.h"

namespace naoqi_msgs
{

  class SetSpeechVocabularyActionGoal : public ros::Msg
  {
    public:
      std_msgs::Header header;
      actionlib_msgs::GoalID goal_id;
      naoqi_msgs::SetSpeechVocabularyGoal goal;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->goal_id.serialize(outbuffer + offset);
      offset += this->goal.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->goal_id.deserialize(inbuffer + offset);
      offset += this->goal.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "naoqi_msgs/SetSpeechVocabularyActionGoal"; };
    const char * getMD5(){ return "f93300199bb500a3e511fde0b0394ddb"; };

  };

}
#endif