#ifndef _ROS_roswifibot_VnQuaternion_msg_h
#define _ROS_roswifibot_VnQuaternion_msg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace roswifibot
{

  class VnQuaternion_msg : public ros::Msg
  {
    public:
      float q0;
      float q1;
      float q2;
      float q3;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_q0;
      u_q0.real = this->q0;
      *(outbuffer + offset + 0) = (u_q0.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_q0.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_q0.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_q0.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->q0);
      union {
        float real;
        uint32_t base;
      } u_q1;
      u_q1.real = this->q1;
      *(outbuffer + offset + 0) = (u_q1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_q1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_q1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_q1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->q1);
      union {
        float real;
        uint32_t base;
      } u_q2;
      u_q2.real = this->q2;
      *(outbuffer + offset + 0) = (u_q2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_q2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_q2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_q2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->q2);
      union {
        float real;
        uint32_t base;
      } u_q3;
      u_q3.real = this->q3;
      *(outbuffer + offset + 0) = (u_q3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_q3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_q3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_q3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->q3);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_q0;
      u_q0.base = 0;
      u_q0.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_q0.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_q0.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_q0.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->q0 = u_q0.real;
      offset += sizeof(this->q0);
      union {
        float real;
        uint32_t base;
      } u_q1;
      u_q1.base = 0;
      u_q1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_q1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_q1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_q1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->q1 = u_q1.real;
      offset += sizeof(this->q1);
      union {
        float real;
        uint32_t base;
      } u_q2;
      u_q2.base = 0;
      u_q2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_q2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_q2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_q2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->q2 = u_q2.real;
      offset += sizeof(this->q2);
      union {
        float real;
        uint32_t base;
      } u_q3;
      u_q3.base = 0;
      u_q3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_q3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_q3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_q3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->q3 = u_q3.real;
      offset += sizeof(this->q3);
     return offset;
    }

    const char * getType(){ return "roswifibot/VnQuaternion_msg"; };
    const char * getMD5(){ return "63ca77594d1bad6ed3441f10f9332674"; };

  };

}
#endif