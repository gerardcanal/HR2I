#ifndef _ROS_SERVICE_AudioMasterVolume_h
#define _ROS_SERVICE_AudioMasterVolume_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Int32.h"

namespace nao_interaction_msgs
{

static const char AUDIOMASTERVOLUME[] = "nao_interaction_msgs/AudioMasterVolume";

  class AudioMasterVolumeRequest : public ros::Msg
  {
    public:
      std_msgs::Int32 master_volume;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->master_volume.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->master_volume.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return AUDIOMASTERVOLUME; };
    const char * getMD5(){ return "6a2d45c5fcf9fd89299667da81b32d64"; };

  };

  class AudioMasterVolumeResponse : public ros::Msg
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

    const char * getType(){ return AUDIOMASTERVOLUME; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class AudioMasterVolume {
    public:
    typedef AudioMasterVolumeRequest Request;
    typedef AudioMasterVolumeResponse Response;
  };

}
#endif
