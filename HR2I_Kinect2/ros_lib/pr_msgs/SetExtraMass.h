#ifndef _ROS_SERVICE_SetExtraMass_h
#define _ROS_SERVICE_SetExtraMass_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr_msgs/MassProperties.h"

namespace pr_msgs
{

static const char SETEXTRAMASS[] = "pr_msgs/SetExtraMass";

  class SetExtraMassRequest : public ros::Msg
  {
    public:
      pr_msgs::MassProperties m;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->m.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->m.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SETEXTRAMASS; };
    const char * getMD5(){ return "711c76cfdcaf7366f06dc155d98f60d1"; };

  };

  class SetExtraMassResponse : public ros::Msg
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

    const char * getType(){ return SETEXTRAMASS; };
    const char * getMD5(){ return "4679398f882e7cbdea165980d3ec2888"; };

  };

  class SetExtraMass {
    public:
    typedef SetExtraMassRequest Request;
    typedef SetExtraMassResponse Response;
  };

}
#endif
