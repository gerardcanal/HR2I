#ifndef _ROS_SERVICE_SetHandTorque_h
#define _ROS_SERVICE_SetHandTorque_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

static const char SETHANDTORQUE[] = "pr_msgs/SetHandTorque";

  class SetHandTorqueRequest : public ros::Msg
  {
    public:
      int32_t initial;
      int32_t sustained;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_initial;
      u_initial.real = this->initial;
      *(outbuffer + offset + 0) = (u_initial.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_initial.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_initial.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_initial.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->initial);
      union {
        int32_t real;
        uint32_t base;
      } u_sustained;
      u_sustained.real = this->sustained;
      *(outbuffer + offset + 0) = (u_sustained.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sustained.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sustained.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sustained.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sustained);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_initial;
      u_initial.base = 0;
      u_initial.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_initial.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_initial.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_initial.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->initial = u_initial.real;
      offset += sizeof(this->initial);
      union {
        int32_t real;
        uint32_t base;
      } u_sustained;
      u_sustained.base = 0;
      u_sustained.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sustained.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sustained.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sustained.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sustained = u_sustained.real;
      offset += sizeof(this->sustained);
     return offset;
    }

    const char * getType(){ return SETHANDTORQUE; };
    const char * getMD5(){ return "161f9c0cf0aa41f1f00bf291ccdff7d0"; };

  };

  class SetHandTorqueResponse : public ros::Msg
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

    const char * getType(){ return SETHANDTORQUE; };
    const char * getMD5(){ return "4679398f882e7cbdea165980d3ec2888"; };

  };

  class SetHandTorque {
    public:
    typedef SetHandTorqueRequest Request;
    typedef SetHandTorqueResponse Response;
  };

}
#endif
