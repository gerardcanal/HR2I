#ifndef _ROS_naoqi_msgs_BlinkResult_h
#define _ROS_naoqi_msgs_BlinkResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace naoqi_msgs
{

  class BlinkResult : public ros::Msg
  {
    public:
      bool still_blinking;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_still_blinking;
      u_still_blinking.real = this->still_blinking;
      *(outbuffer + offset + 0) = (u_still_blinking.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->still_blinking);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_still_blinking;
      u_still_blinking.base = 0;
      u_still_blinking.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->still_blinking = u_still_blinking.real;
      offset += sizeof(this->still_blinking);
     return offset;
    }

    const char * getType(){ return "naoqi_msgs/BlinkResult"; };
    const char * getMD5(){ return "53e041b81450f9247036f13b3c0bf822"; };

  };

}
#endif