#ifndef _ROS_SERVICE_WamRequestSeaCtrlKp_h
#define _ROS_SERVICE_WamRequestSeaCtrlKp_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

static const char WAMREQUESTSEACTRLKP[] = "pr_msgs/WamRequestSeaCtrlKp";

  class WamRequestSeaCtrlKpRequest : public ros::Msg
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

    const char * getType(){ return WAMREQUESTSEACTRLKP; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class WamRequestSeaCtrlKpResponse : public ros::Msg
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

    const char * getType(){ return WAMREQUESTSEACTRLKP; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class WamRequestSeaCtrlKp {
    public:
    typedef WamRequestSeaCtrlKpRequest Request;
    typedef WamRequestSeaCtrlKpResponse Response;
  };

}
#endif
