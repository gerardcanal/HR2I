#ifndef _ROS_naoqi_msgs_SpeechWithFeedbackGoal_h
#define _ROS_naoqi_msgs_SpeechWithFeedbackGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace naoqi_msgs
{

  class SpeechWithFeedbackGoal : public ros::Msg
  {
    public:
      const char* say;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_say = strlen(this->say);
      memcpy(outbuffer + offset, &length_say, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->say, length_say);
      offset += length_say;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_say;
      memcpy(&length_say, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_say; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_say-1]=0;
      this->say = (char *)(inbuffer + offset-1);
      offset += length_say;
     return offset;
    }

    const char * getType(){ return "naoqi_msgs/SpeechWithFeedbackGoal"; };
    const char * getMD5(){ return "331898fd34308d7c3706d43ca7f6e377"; };

  };

}
#endif