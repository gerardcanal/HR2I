#ifndef _ROS_pr_msgs_MaglevFeedback_h
#define _ROS_pr_msgs_MaglevFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

  class MaglevFeedback : public ros::Msg
  {
    public:
      uint8_t mode;
      uint8_t value_length;
      float st_value;
      float * value;
      enum { POSITION = 0 };
      enum { FORCE = 1 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mode);
      *(outbuffer + offset++) = value_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < value_length; i++){
      union {
        float real;
        uint32_t base;
      } u_valuei;
      u_valuei.real = this->value[i];
      *(outbuffer + offset + 0) = (u_valuei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_valuei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_valuei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_valuei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->value[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->mode);
      uint8_t value_lengthT = *(inbuffer + offset++);
      if(value_lengthT > value_length)
        this->value = (float*)realloc(this->value, value_lengthT * sizeof(float));
      offset += 3;
      value_length = value_lengthT;
      for( uint8_t i = 0; i < value_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_value;
      u_st_value.base = 0;
      u_st_value.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_value.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_value.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_value.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_value = u_st_value.real;
      offset += sizeof(this->st_value);
        memcpy( &(this->value[i]), &(this->st_value), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "pr_msgs/MaglevFeedback"; };
    const char * getMD5(){ return "def18e2e7b5c4406e703d277c2474767"; };

  };

}
#endif