#ifndef _ROS_SERVICE_Enable_h
#define _ROS_SERVICE_Enable_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

static const char ENABLE[] = "pr_msgs/Enable";

  class EnableRequest : public ros::Msg
  {
    public:
      bool Enable;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_Enable;
      u_Enable.real = this->Enable;
      *(outbuffer + offset + 0) = (u_Enable.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Enable);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_Enable;
      u_Enable.base = 0;
      u_Enable.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->Enable = u_Enable.real;
      offset += sizeof(this->Enable);
     return offset;
    }

    const char * getType(){ return ENABLE; };
    const char * getMD5(){ return "132b53c6b897b73e7dc72146d30f3b1e"; };

  };

  class EnableResponse : public ros::Msg
  {
    public:
      bool ok;
      char * reason;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ok;
      u_ok.real = this->ok;
      *(outbuffer + offset + 0) = (u_ok.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ok);
      uint32_t length_reason = strlen( (const char*) this->reason);
      memcpy(outbuffer + offset, &length_reason, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->reason, length_reason);
      offset += length_reason;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ok;
      u_ok.base = 0;
      u_ok.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ok = u_ok.real;
      offset += sizeof(this->ok);
      uint32_t length_reason;
      memcpy(&length_reason, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_reason; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_reason-1]=0;
      this->reason = (char *)(inbuffer + offset-1);
      offset += length_reason;
     return offset;
    }

    const char * getType(){ return ENABLE; };
    const char * getMD5(){ return "4679398f882e7cbdea165980d3ec2888"; };

  };

  class Enable {
    public:
    typedef EnableRequest Request;
    typedef EnableResponse Response;
  };

}
#endif
