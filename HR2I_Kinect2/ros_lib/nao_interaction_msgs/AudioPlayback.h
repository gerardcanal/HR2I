#ifndef _ROS_SERVICE_AudioPlayback_h
#define _ROS_SERVICE_AudioPlayback_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/String.h"

namespace nao_interaction_msgs
{

static const char AUDIOPLAYBACK[] = "nao_interaction_msgs/AudioPlayback";

  class AudioPlaybackRequest : public ros::Msg
  {
    public:
      std_msgs::String file_path;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->file_path.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->file_path.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return AUDIOPLAYBACK; };
    const char * getMD5(){ return "0bc1b80cbf52b04c96479758726740cc"; };

  };

  class AudioPlaybackResponse : public ros::Msg
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

    const char * getType(){ return AUDIOPLAYBACK; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class AudioPlayback {
    public:
    typedef AudioPlaybackRequest Request;
    typedef AudioPlaybackResponse Response;
  };

}
#endif
