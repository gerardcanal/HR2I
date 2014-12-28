#ifndef _ROS_SERVICE_ResetAkinator_h
#define _ROS_SERVICE_ResetAkinator_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace naokinator_ros
{

static const char RESETAKINATOR[] = "naokinator_ros/ResetAkinator";

  class ResetAkinatorRequest : public ros::Msg
  {
    public:
      const char* name;
      int8_t age;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      memcpy(outbuffer + offset, &length_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      union {
        int8_t real;
        uint8_t base;
      } u_age;
      u_age.real = this->age;
      *(outbuffer + offset + 0) = (u_age.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->age);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      memcpy(&length_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      union {
        int8_t real;
        uint8_t base;
      } u_age;
      u_age.base = 0;
      u_age.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->age = u_age.real;
      offset += sizeof(this->age);
     return offset;
    }

    const char * getType(){ return RESETAKINATOR; };
    const char * getMD5(){ return "23b9bccd17c8042888a78e4f556ccd59"; };

  };

  class ResetAkinatorResponse : public ros::Msg
  {
    public:

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return RESETAKINATOR; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ResetAkinator {
    public:
    typedef ResetAkinatorRequest Request;
    typedef ResetAkinatorResponse Response;
  };

}
#endif
