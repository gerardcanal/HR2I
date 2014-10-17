#ifndef _ROS_SERVICE_StepJoint_h
#define _ROS_SERVICE_StepJoint_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

static const char STEPJOINT[] = "pr_msgs/StepJoint";

  class StepJointRequest : public ros::Msg
  {
    public:
      uint8_t joint;
      double radians;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->joint >> (8 * 0)) & 0xFF;
      offset += sizeof(this->joint);
      union {
        double real;
        uint64_t base;
      } u_radians;
      u_radians.real = this->radians;
      *(outbuffer + offset + 0) = (u_radians.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_radians.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_radians.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_radians.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_radians.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_radians.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_radians.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_radians.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->radians);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->joint =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->joint);
      union {
        double real;
        uint64_t base;
      } u_radians;
      u_radians.base = 0;
      u_radians.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_radians.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_radians.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_radians.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_radians.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_radians.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_radians.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_radians.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->radians = u_radians.real;
      offset += sizeof(this->radians);
     return offset;
    }

    const char * getType(){ return STEPJOINT; };
    const char * getMD5(){ return "e76fd844c151f2b85b89aba56e105bdc"; };

  };

  class StepJointResponse : public ros::Msg
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

    const char * getType(){ return STEPJOINT; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class StepJoint {
    public:
    typedef StepJointRequest Request;
    typedef StepJointResponse Response;
  };

}
#endif
