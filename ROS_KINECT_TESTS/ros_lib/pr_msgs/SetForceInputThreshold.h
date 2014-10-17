#ifndef _ROS_SERVICE_SetForceInputThreshold_h
#define _ROS_SERVICE_SetForceInputThreshold_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace pr_msgs
{

static const char SETFORCEINPUTTHRESHOLD[] = "pr_msgs/SetForceInputThreshold";

  class SetForceInputThresholdRequest : public ros::Msg
  {
    public:
      geometry_msgs::Vector3 direction;
      double force;
      geometry_msgs::Vector3 torques;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->direction.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_force;
      u_force.real = this->force;
      *(outbuffer + offset + 0) = (u_force.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_force.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_force.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_force.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_force.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_force.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_force.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_force.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->force);
      offset += this->torques.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->direction.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_force;
      u_force.base = 0;
      u_force.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_force.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_force.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_force.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_force.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_force.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_force.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_force.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->force = u_force.real;
      offset += sizeof(this->force);
      offset += this->torques.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SETFORCEINPUTTHRESHOLD; };
    const char * getMD5(){ return "667c67fac1f7f607f50ecfa816afa76d"; };

  };

  class SetForceInputThresholdResponse : public ros::Msg
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

    const char * getType(){ return SETFORCEINPUTTHRESHOLD; };
    const char * getMD5(){ return "4679398f882e7cbdea165980d3ec2888"; };

  };

  class SetForceInputThreshold {
    public:
    typedef SetForceInputThresholdRequest Request;
    typedef SetForceInputThresholdResponse Response;
  };

}
#endif
