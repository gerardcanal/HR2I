#ifndef _ROS_SERVICE_akinator_srv_h
#define _ROS_SERVICE_akinator_srv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace naokinator_ros
{

static const char AKINATOR_SRV[] = "naokinator_ros/akinator_srv";

  class akinator_srvRequest : public ros::Msg
  {
    public:
      const char* question_response;
      enum { init_conversation =  "initialise conversation" };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_question_response = strlen(this->question_response);
      memcpy(outbuffer + offset, &length_question_response, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->question_response, length_question_response);
      offset += length_question_response;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_question_response;
      memcpy(&length_question_response, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_question_response; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_question_response-1]=0;
      this->question_response = (char *)(inbuffer + offset-1);
      offset += length_question_response;
     return offset;
    }

    const char * getType(){ return AKINATOR_SRV; };
    const char * getMD5(){ return "76b1588506eeb3580fd4917b824208b2"; };

  };

  class akinator_srvResponse : public ros::Msg
  {
    public:
      const char* question_guess_response;
      bool is_guess;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_question_guess_response = strlen(this->question_guess_response);
      memcpy(outbuffer + offset, &length_question_guess_response, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->question_guess_response, length_question_guess_response);
      offset += length_question_guess_response;
      union {
        bool real;
        uint8_t base;
      } u_is_guess;
      u_is_guess.real = this->is_guess;
      *(outbuffer + offset + 0) = (u_is_guess.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_guess);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_question_guess_response;
      memcpy(&length_question_guess_response, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_question_guess_response; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_question_guess_response-1]=0;
      this->question_guess_response = (char *)(inbuffer + offset-1);
      offset += length_question_guess_response;
      union {
        bool real;
        uint8_t base;
      } u_is_guess;
      u_is_guess.base = 0;
      u_is_guess.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_guess = u_is_guess.real;
      offset += sizeof(this->is_guess);
     return offset;
    }

    const char * getType(){ return AKINATOR_SRV; };
    const char * getMD5(){ return "d3d2ac16e7913d4d0c465ae3e39e1fe4"; };

  };

  class akinator_srv {
    public:
    typedef akinator_srvRequest Request;
    typedef akinator_srvResponse Response;
  };

}
#endif
