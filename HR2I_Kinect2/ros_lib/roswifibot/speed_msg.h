#ifndef _ROS_roswifibot_speed_msg_h
#define _ROS_roswifibot_speed_msg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace roswifibot
{

  class speed_msg : public ros::Msg
  {
    public:
      float speedLeft;
      float speedRight;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_speedLeft;
      u_speedLeft.real = this->speedLeft;
      *(outbuffer + offset + 0) = (u_speedLeft.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speedLeft.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speedLeft.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speedLeft.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speedLeft);
      union {
        float real;
        uint32_t base;
      } u_speedRight;
      u_speedRight.real = this->speedRight;
      *(outbuffer + offset + 0) = (u_speedRight.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speedRight.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speedRight.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speedRight.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speedRight);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_speedLeft;
      u_speedLeft.base = 0;
      u_speedLeft.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speedLeft.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speedLeft.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speedLeft.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speedLeft = u_speedLeft.real;
      offset += sizeof(this->speedLeft);
      union {
        float real;
        uint32_t base;
      } u_speedRight;
      u_speedRight.base = 0;
      u_speedRight.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speedRight.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speedRight.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speedRight.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speedRight = u_speedRight.real;
      offset += sizeof(this->speedRight);
     return offset;
    }

    const char * getType(){ return "roswifibot/speed_msg"; };
    const char * getMD5(){ return "c26338313736f55e11b890e5bc4e2030"; };

  };

}
#endif