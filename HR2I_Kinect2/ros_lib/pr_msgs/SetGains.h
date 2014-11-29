#ifndef _ROS_SERVICE_SetGains_h
#define _ROS_SERVICE_SetGains_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr_msgs/PIDgains.h"

namespace pr_msgs
{

static const char SETGAINS[] = "pr_msgs/SetGains";

  class SetGainsRequest : public ros::Msg
  {
    public:
      uint8_t joint;
      pr_msgs::PIDgains gains;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->joint >> (8 * 0)) & 0xFF;
      offset += sizeof(this->joint);
      offset += this->gains.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->joint =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->joint);
      offset += this->gains.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SETGAINS; };
    const char * getMD5(){ return "8e1b236ea8d7f7d7cef0473991af0ae8"; };

  };

  class SetGainsResponse : public ros::Msg
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

    const char * getType(){ return SETGAINS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetGains {
    public:
    typedef SetGainsRequest Request;
    typedef SetGainsResponse Response;
  };

}
#endif
