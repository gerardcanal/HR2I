#ifndef _ROS_SERVICE_CalibrateJoints_h
#define _ROS_SERVICE_CalibrateJoints_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

static const char CALIBRATEJOINTS[] = "pr_msgs/CalibrateJoints";

  class CalibrateJointsRequest : public ros::Msg
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

    const char * getType(){ return CALIBRATEJOINTS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class CalibrateJointsResponse : public ros::Msg
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

    const char * getType(){ return CALIBRATEJOINTS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class CalibrateJoints {
    public:
    typedef CalibrateJointsRequest Request;
    typedef CalibrateJointsResponse Response;
  };

}
#endif
