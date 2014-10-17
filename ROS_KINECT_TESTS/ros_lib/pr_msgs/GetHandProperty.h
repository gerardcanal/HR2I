#ifndef _ROS_SERVICE_GetHandProperty_h
#define _ROS_SERVICE_GetHandProperty_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

static const char GETHANDPROPERTY[] = "pr_msgs/GetHandProperty";

  class GetHandPropertyRequest : public ros::Msg
  {
    public:
      int32_t nodeid;
      int32_t property;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_nodeid;
      u_nodeid.real = this->nodeid;
      *(outbuffer + offset + 0) = (u_nodeid.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_nodeid.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_nodeid.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_nodeid.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nodeid);
      union {
        int32_t real;
        uint32_t base;
      } u_property;
      u_property.real = this->property;
      *(outbuffer + offset + 0) = (u_property.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_property.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_property.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_property.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->property);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_nodeid;
      u_nodeid.base = 0;
      u_nodeid.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_nodeid.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_nodeid.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_nodeid.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->nodeid = u_nodeid.real;
      offset += sizeof(this->nodeid);
      union {
        int32_t real;
        uint32_t base;
      } u_property;
      u_property.base = 0;
      u_property.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_property.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_property.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_property.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->property = u_property.real;
      offset += sizeof(this->property);
     return offset;
    }

    const char * getType(){ return GETHANDPROPERTY; };
    const char * getMD5(){ return "9ed9d5b98f25032a6d549c1cb96e061b"; };

  };

  class GetHandPropertyResponse : public ros::Msg
  {
    public:
      int32_t value;
      bool ok;
      char * reason;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_value;
      u_value.real = this->value;
      *(outbuffer + offset + 0) = (u_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_value.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_value.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_value.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->value);
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
        int32_t real;
        uint32_t base;
      } u_value;
      u_value.base = 0;
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->value = u_value.real;
      offset += sizeof(this->value);
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

    const char * getType(){ return GETHANDPROPERTY; };
    const char * getMD5(){ return "1a586ea1d6033f5c3d181e12f9b4e533"; };

  };

  class GetHandProperty {
    public:
    typedef GetHandPropertyRequest Request;
    typedef GetHandPropertyResponse Response;
  };

}
#endif
