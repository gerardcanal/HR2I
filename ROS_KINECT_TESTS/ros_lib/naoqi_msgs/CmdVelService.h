#ifndef _ROS_SERVICE_CmdVelService_h
#define _ROS_SERVICE_CmdVelService_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Twist.h"

namespace naoqi_msgs
{

static const char CMDVELSERVICE[] = "naoqi_msgs/CmdVelService";

  class CmdVelServiceRequest : public ros::Msg
  {
    public:
      geometry_msgs::Twist twist;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->twist.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->twist.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return CMDVELSERVICE; };
    const char * getMD5(){ return "a787b2802160dcc7fe02d089e10afe56"; };

  };

  class CmdVelServiceResponse : public ros::Msg
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

    const char * getType(){ return CMDVELSERVICE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class CmdVelService {
    public:
    typedef CmdVelServiceRequest Request;
    typedef CmdVelServiceResponse Response;
  };

}
#endif
