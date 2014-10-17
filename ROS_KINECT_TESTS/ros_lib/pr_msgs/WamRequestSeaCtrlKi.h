#ifndef _ROS_SERVICE_WamRequestSeaCtrlKi_h
#define _ROS_SERVICE_WamRequestSeaCtrlKi_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

static const char WAMREQUESTSEACTRLKI[] = "pr_msgs/WamRequestSeaCtrlKi";

  class WamRequestSeaCtrlKiRequest : public ros::Msg
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

    const char * getType(){ return WAMREQUESTSEACTRLKI; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class WamRequestSeaCtrlKiResponse : public ros::Msg
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

    const char * getType(){ return WAMREQUESTSEACTRLKI; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class WamRequestSeaCtrlKi {
    public:
    typedef WamRequestSeaCtrlKiRequest Request;
    typedef WamRequestSeaCtrlKiResponse Response;
  };

}
#endif
