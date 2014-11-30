#ifndef _ROS_roswifibot_VnVector3_msg_h
#define _ROS_roswifibot_VnVector3_msg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace roswifibot
{

  class VnVector3_msg : public ros::Msg
  {
    public:
      double c0;
      double c1;
      double c2;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_c0;
      u_c0.real = this->c0;
      *(outbuffer + offset + 0) = (u_c0.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_c0.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_c0.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_c0.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_c0.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_c0.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_c0.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_c0.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->c0);
      union {
        double real;
        uint64_t base;
      } u_c1;
      u_c1.real = this->c1;
      *(outbuffer + offset + 0) = (u_c1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_c1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_c1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_c1.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_c1.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_c1.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_c1.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_c1.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->c1);
      union {
        double real;
        uint64_t base;
      } u_c2;
      u_c2.real = this->c2;
      *(outbuffer + offset + 0) = (u_c2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_c2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_c2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_c2.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_c2.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_c2.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_c2.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_c2.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->c2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_c0;
      u_c0.base = 0;
      u_c0.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_c0.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_c0.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_c0.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_c0.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_c0.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_c0.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_c0.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->c0 = u_c0.real;
      offset += sizeof(this->c0);
      union {
        double real;
        uint64_t base;
      } u_c1;
      u_c1.base = 0;
      u_c1.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_c1.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_c1.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_c1.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_c1.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_c1.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_c1.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_c1.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->c1 = u_c1.real;
      offset += sizeof(this->c1);
      union {
        double real;
        uint64_t base;
      } u_c2;
      u_c2.base = 0;
      u_c2.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_c2.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_c2.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_c2.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_c2.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_c2.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_c2.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_c2.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->c2 = u_c2.real;
      offset += sizeof(this->c2);
     return offset;
    }

    const char * getType(){ return "roswifibot/VnVector3_msg"; };
    const char * getMD5(){ return "61a8bcaff34ca06d673fe6fdbe9a80c5"; };

  };

}
#endif