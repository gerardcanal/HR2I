#ifndef _ROS_SERVICE_LearnFace_h
#define _ROS_SERVICE_LearnFace_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

namespace nao_interaction_msgs
{

static const char LEARNFACE[] = "nao_interaction_msgs/LearnFace";

  class LearnFaceRequest : public ros::Msg
  {
    public:
      std_msgs::String name;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->name.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->name.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return LEARNFACE; };
    const char * getMD5(){ return "0fce35bd9f5b27a63eb9b0e831759a0b"; };

  };

  class LearnFaceResponse : public ros::Msg
  {
    public:
      std_msgs::Bool result;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return LEARNFACE; };
    const char * getMD5(){ return "c2420602a83d8ccc0f3664e0daafab6c"; };

  };

  class LearnFace {
    public:
    typedef LearnFaceRequest Request;
    typedef LearnFaceResponse Response;
  };

}
#endif
