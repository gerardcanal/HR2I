#ifndef _ROS_SERVICE_enable_h
#define _ROS_SERVICE_enable_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace object_recognition_mock
{

static const char ENABLE[] = "object_recognition_mock/enable";

  class enableRequest : public ros::Msg
  {
    public:
      bool enable;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_enable;
      u_enable.real = this->enable;
      *(outbuffer + offset + 0) = (u_enable.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enable);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_enable;
      u_enable.base = 0;
      u_enable.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enable = u_enable.real;
      offset += sizeof(this->enable);
     return offset;
    }

    const char * getType(){ return ENABLE; };
    const char * getMD5(){ return "8c1211af706069c994c06e00eb59ac9e"; };

  };

  class enableResponse : public ros::Msg
  {
    public:
      bool correct;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_correct;
      u_correct.real = this->correct;
      *(outbuffer + offset + 0) = (u_correct.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->correct);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_correct;
      u_correct.base = 0;
      u_correct.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->correct = u_correct.real;
      offset += sizeof(this->correct);
     return offset;
    }

    const char * getType(){ return ENABLE; };
    const char * getMD5(){ return "0d7b90c75811aaad705aac4e2b606238"; };

  };

  class enable {
    public:
    typedef enableRequest Request;
    typedef enableResponse Response;
  };

}
#endif
