#ifndef _ROS_SERVICE_AudioRecorder_h
#define _ROS_SERVICE_AudioRecorder_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Char.h"
#include "std_msgs/Int32.h"

namespace nao_interaction_msgs
{

static const char AUDIORECORDER[] = "nao_interaction_msgs/AudioRecorder";

  class AudioRecorderRequest : public ros::Msg
  {
    public:
      std_msgs::String file_path;
      std_msgs::Int32 secs;
      std_msgs::Char audio_type;
      std_msgs::Bool left_channel;
      std_msgs::Bool right_channel;
      std_msgs::Bool front_channel;
      std_msgs::Bool rear_channel;
      std_msgs::Int32 samplerate;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->file_path.serialize(outbuffer + offset);
      offset += this->secs.serialize(outbuffer + offset);
      offset += this->audio_type.serialize(outbuffer + offset);
      offset += this->left_channel.serialize(outbuffer + offset);
      offset += this->right_channel.serialize(outbuffer + offset);
      offset += this->front_channel.serialize(outbuffer + offset);
      offset += this->rear_channel.serialize(outbuffer + offset);
      offset += this->samplerate.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->file_path.deserialize(inbuffer + offset);
      offset += this->secs.deserialize(inbuffer + offset);
      offset += this->audio_type.deserialize(inbuffer + offset);
      offset += this->left_channel.deserialize(inbuffer + offset);
      offset += this->right_channel.deserialize(inbuffer + offset);
      offset += this->front_channel.deserialize(inbuffer + offset);
      offset += this->rear_channel.deserialize(inbuffer + offset);
      offset += this->samplerate.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return AUDIORECORDER; };
    const char * getMD5(){ return "b211df69b24b5ed7b95654b151a08b5c"; };

  };

  class AudioRecorderResponse : public ros::Msg
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

    const char * getType(){ return AUDIORECORDER; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class AudioRecorder {
    public:
    typedef AudioRecorderRequest Request;
    typedef AudioRecorderResponse Response;
  };

}
#endif
