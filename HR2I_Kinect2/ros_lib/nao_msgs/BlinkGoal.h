#ifndef _ROS_nao_msgs_BlinkGoal_h
#define _ROS_nao_msgs_BlinkGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/ColorRGBA.h"
#include "ros/duration.h"

namespace nao_msgs
{

  class BlinkGoal : public ros::Msg
  {
    public:
      uint8_t colors_length;
      std_msgs::ColorRGBA st_colors;
      std_msgs::ColorRGBA * colors;
      std_msgs::ColorRGBA bg_color;
      ros::Duration blink_duration;
      float blink_rate_mean;
      float blink_rate_sd;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = colors_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < colors_length; i++){
      offset += this->colors[i].serialize(outbuffer + offset);
      }
      offset += this->bg_color.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->blink_duration.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->blink_duration.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->blink_duration.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->blink_duration.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->blink_duration.sec);
      *(outbuffer + offset + 0) = (this->blink_duration.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->blink_duration.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->blink_duration.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->blink_duration.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->blink_duration.nsec);
      union {
        float real;
        uint32_t base;
      } u_blink_rate_mean;
      u_blink_rate_mean.real = this->blink_rate_mean;
      *(outbuffer + offset + 0) = (u_blink_rate_mean.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_blink_rate_mean.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_blink_rate_mean.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_blink_rate_mean.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->blink_rate_mean);
      union {
        float real;
        uint32_t base;
      } u_blink_rate_sd;
      u_blink_rate_sd.real = this->blink_rate_sd;
      *(outbuffer + offset + 0) = (u_blink_rate_sd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_blink_rate_sd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_blink_rate_sd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_blink_rate_sd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->blink_rate_sd);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t colors_lengthT = *(inbuffer + offset++);
      if(colors_lengthT > colors_length)
        this->colors = (std_msgs::ColorRGBA*)realloc(this->colors, colors_lengthT * sizeof(std_msgs::ColorRGBA));
      offset += 3;
      colors_length = colors_lengthT;
      for( uint8_t i = 0; i < colors_length; i++){
      offset += this->st_colors.deserialize(inbuffer + offset);
        memcpy( &(this->colors[i]), &(this->st_colors), sizeof(std_msgs::ColorRGBA));
      }
      offset += this->bg_color.deserialize(inbuffer + offset);
      this->blink_duration.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->blink_duration.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->blink_duration.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->blink_duration.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->blink_duration.sec);
      this->blink_duration.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->blink_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->blink_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->blink_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->blink_duration.nsec);
      union {
        float real;
        uint32_t base;
      } u_blink_rate_mean;
      u_blink_rate_mean.base = 0;
      u_blink_rate_mean.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_blink_rate_mean.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_blink_rate_mean.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_blink_rate_mean.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->blink_rate_mean = u_blink_rate_mean.real;
      offset += sizeof(this->blink_rate_mean);
      union {
        float real;
        uint32_t base;
      } u_blink_rate_sd;
      u_blink_rate_sd.base = 0;
      u_blink_rate_sd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_blink_rate_sd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_blink_rate_sd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_blink_rate_sd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->blink_rate_sd = u_blink_rate_sd.real;
      offset += sizeof(this->blink_rate_sd);
     return offset;
    }

    const char * getType(){ return "nao_msgs/BlinkGoal"; };
    const char * getMD5(){ return "5e5d3c2ba9976dc121a0bb6ef7c66d79"; };

  };

}
#endif