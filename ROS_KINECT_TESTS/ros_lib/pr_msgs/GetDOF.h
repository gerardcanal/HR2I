#ifndef _ROS_SERVICE_GetDOF_h
#define _ROS_SERVICE_GetDOF_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

static const char GETDOF[] = "pr_msgs/GetDOF";

  class GetDOFRequest : public ros::Msg
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

    const char * getType(){ return GETDOF; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetDOFResponse : public ros::Msg
  {
    public:
      uint32_t nDOF;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->nDOF >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nDOF >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->nDOF >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->nDOF >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nDOF);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->nDOF =  ((uint32_t) (*(inbuffer + offset)));
      this->nDOF |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->nDOF |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->nDOF |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->nDOF);
     return offset;
    }

    const char * getType(){ return GETDOF; };
    const char * getMD5(){ return "28965d9e0d5ec6fa5ecf9e0da0bee01d"; };

  };

  class GetDOF {
    public:
    typedef GetDOFRequest Request;
    typedef GetDOFResponse Response;
  };

}
#endif
