#ifndef _ROS_text_to_speech_SoundGoal_h
#define _ROS_text_to_speech_SoundGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/duration.h"

namespace text_to_speech
{

  class SoundGoal : public ros::Msg
  {
    public:
      char * text;
      ros::Duration wait_before_speaking;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_text = strlen( (const char*) this->text);
      memcpy(outbuffer + offset, &length_text, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->text, length_text);
      offset += length_text;
      *(outbuffer + offset + 0) = (this->wait_before_speaking.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->wait_before_speaking.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->wait_before_speaking.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->wait_before_speaking.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wait_before_speaking.sec);
      *(outbuffer + offset + 0) = (this->wait_before_speaking.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->wait_before_speaking.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->wait_before_speaking.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->wait_before_speaking.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wait_before_speaking.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_text;
      memcpy(&length_text, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_text; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_text-1]=0;
      this->text = (char *)(inbuffer + offset-1);
      offset += length_text;
      this->wait_before_speaking.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->wait_before_speaking.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->wait_before_speaking.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->wait_before_speaking.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->wait_before_speaking.sec);
      this->wait_before_speaking.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->wait_before_speaking.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->wait_before_speaking.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->wait_before_speaking.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->wait_before_speaking.nsec);
     return offset;
    }

    const char * getType(){ return "text_to_speech/SoundGoal"; };
    const char * getMD5(){ return "07b6eb0f0c772a431afd552c46d51064"; };

  };

}
#endif