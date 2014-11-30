#ifndef _ROS_nao_interaction_msgs_AudioSourceLocalization_h
#define _ROS_nao_interaction_msgs_AudioSourceLocalization_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose.h"

namespace nao_interaction_msgs
{

  class AudioSourceLocalization : public ros::Msg
  {
    public:
      std_msgs::Header header;
      std_msgs::Float32 azimuth;
      std_msgs::Float32 elevation;
      std_msgs::Float32 confidence;
      std_msgs::Float32 energy;
      geometry_msgs::Pose head_pose;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->azimuth.serialize(outbuffer + offset);
      offset += this->elevation.serialize(outbuffer + offset);
      offset += this->confidence.serialize(outbuffer + offset);
      offset += this->energy.serialize(outbuffer + offset);
      offset += this->head_pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->azimuth.deserialize(inbuffer + offset);
      offset += this->elevation.deserialize(inbuffer + offset);
      offset += this->confidence.deserialize(inbuffer + offset);
      offset += this->energy.deserialize(inbuffer + offset);
      offset += this->head_pose.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "nao_interaction_msgs/AudioSourceLocalization"; };
    const char * getMD5(){ return "5c65a283c3f4c0f07561982b3d8c4f13"; };

  };

}
#endif