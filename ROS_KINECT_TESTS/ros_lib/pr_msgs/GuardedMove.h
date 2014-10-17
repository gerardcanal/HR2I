#ifndef _ROS_SERVICE_GuardedMove_h
#define _ROS_SERVICE_GuardedMove_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr_msgs/Joints.h"

namespace pr_msgs
{

static const char GUARDEDMOVE[] = "pr_msgs/GuardedMove";

  class GuardedMoveRequest : public ros::Msg
  {
    public:
      pr_msgs::Joints positions;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->positions.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->positions.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GUARDEDMOVE; };
    const char * getMD5(){ return "652ae8faea68fe21ca4074dd9f25e569"; };

  };

  class GuardedMoveResponse : public ros::Msg
  {
    public:
      bool ok;

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
     return offset;
    }

    const char * getType(){ return GUARDEDMOVE; };
    const char * getMD5(){ return "6f6da3883749771fac40d6deb24a8c02"; };

  };

  class GuardedMove {
    public:
    typedef GuardedMoveRequest Request;
    typedef GuardedMoveResponse Response;
  };

}
#endif
