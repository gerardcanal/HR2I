#ifndef _ROS_naoqi_msgs_AudioBuffer_h
#define _ROS_naoqi_msgs_AudioBuffer_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace naoqi_msgs
{

  class AudioBuffer : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint16_t frequency;
      uint8_t channelMap_length;
      uint8_t st_channelMap;
      uint8_t * channelMap;
      uint8_t data_length;
      int16_t st_data;
      int16_t * data;
      enum { CHANNEL_FRONT_LEFT = 0 };
      enum { CHANNEL_FRONT_CENTER = 1 };
      enum { CHANNEL_FRONT_RIGHT = 2 };
      enum { CHANNEL_REAR_LEFT = 3 };
      enum { CHANNEL_REAR_CENTER = 4 };
      enum { CHANNEL_REAR_RIGHT = 5 };
      enum { CHANNEL_SURROUND_LEFT = 6 };
      enum { CHANNEL_SURROUND_RIGHT = 7 };
      enum { CHANNEL_SUBWOOFER = 8 };
      enum { CHANNEL_LFE = 9 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->frequency >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->frequency >> (8 * 1)) & 0xFF;
      offset += sizeof(this->frequency);
      *(outbuffer + offset++) = channelMap_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < channelMap_length; i++){
      *(outbuffer + offset + 0) = (this->channelMap[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->channelMap[i]);
      }
      *(outbuffer + offset++) = data_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < data_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_datai.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->frequency =  ((uint16_t) (*(inbuffer + offset)));
      this->frequency |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->frequency);
      uint8_t channelMap_lengthT = *(inbuffer + offset++);
      if(channelMap_lengthT > channelMap_length)
        this->channelMap = (uint8_t*)realloc(this->channelMap, channelMap_lengthT * sizeof(uint8_t));
      offset += 3;
      channelMap_length = channelMap_lengthT;
      for( uint8_t i = 0; i < channelMap_length; i++){
      this->st_channelMap =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_channelMap);
        memcpy( &(this->channelMap[i]), &(this->st_channelMap), sizeof(uint8_t));
      }
      uint8_t data_lengthT = *(inbuffer + offset++);
      if(data_lengthT > data_length)
        this->data = (int16_t*)realloc(this->data, data_lengthT * sizeof(int16_t));
      offset += 3;
      data_length = data_lengthT;
      for( uint8_t i = 0; i < data_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_st_data;
      u_st_data.base = 0;
      u_st_data.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_data.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_data = u_st_data.real;
      offset += sizeof(this->st_data);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(int16_t));
      }
     return offset;
    }

    const char * getType(){ return "naoqi_msgs/AudioBuffer"; };
    const char * getMD5(){ return "50f300aa63f3c1b2f3d3173329165316"; };

  };

}
#endif