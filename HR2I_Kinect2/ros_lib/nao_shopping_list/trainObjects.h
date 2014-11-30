#ifndef _ROS_SERVICE_trainObjects_h
#define _ROS_SERVICE_trainObjects_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace nao_shopping_list
{

static const char TRAINOBJECTS[] = "nao_shopping_list/trainObjects";

  class trainObjectsRequest : public ros::Msg
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

    const char * getType(){ return TRAINOBJECTS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class trainObjectsResponse : public ros::Msg
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

    const char * getType(){ return TRAINOBJECTS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class trainObjects {
    public:
    typedef trainObjectsRequest Request;
    typedef trainObjectsResponse Response;
  };

}
#endif
