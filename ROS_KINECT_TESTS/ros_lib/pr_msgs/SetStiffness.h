#ifndef _ROS_SERVICE_SetStiffness_h
#define _ROS_SERVICE_SetStiffness_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

static const char SETSTIFFNESS[] = "pr_msgs/SetStiffness";

  class SetStiffnessRequest : public ros::Msg
  {
    public:
      float stiffness;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_stiffness;
      u_stiffness.real = this->stiffness;
      *(outbuffer + offset + 0) = (u_stiffness.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_stiffness.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_stiffness.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_stiffness.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stiffness);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_stiffness;
      u_stiffness.base = 0;
      u_stiffness.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_stiffness.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_stiffness.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_stiffness.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->stiffness = u_stiffness.real;
      offset += sizeof(this->stiffness);
     return offset;
    }

    const char * getType(){ return SETSTIFFNESS; };
    const char * getMD5(){ return "209e95af5ab4c92aa955d8dff6cb6543"; };

  };

  class SetStiffnessResponse : public ros::Msg
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

    const char * getType(){ return SETSTIFFNESS; };
    const char * getMD5(){ return "4679398f882e7cbdea165980d3ec2888"; };

  };

  class SetStiffness {
    public:
    typedef SetStiffnessRequest Request;
    typedef SetStiffnessResponse Response;
  };

}
#endif
