#ifndef _ROS_SERVICE_recognizerService_h
#define _ROS_SERVICE_recognizerService_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pal_interaction_msgs/asrupdate.h"

namespace pal_interaction_msgs
{

static const char RECOGNIZERSERVICE[] = "pal_interaction_msgs/recognizerService";

  class recognizerServiceRequest : public ros::Msg
  {
    public:
      pal_interaction_msgs::asrupdate asrupdate;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->asrupdate.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->asrupdate.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return RECOGNIZERSERVICE; };
    const char * getMD5(){ return "cfe4314795c9fe304282fdbecf5be6ec"; };

  };

  class recognizerServiceResponse : public ros::Msg
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

    const char * getType(){ return RECOGNIZERSERVICE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class recognizerService {
    public:
    typedef recognizerServiceRequest Request;
    typedef recognizerServiceResponse Response;
  };

}
#endif
