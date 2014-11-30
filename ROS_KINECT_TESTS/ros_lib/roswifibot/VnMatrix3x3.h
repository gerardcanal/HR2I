#ifndef _ROS_roswifibot_VnMatrix3x3_h
#define _ROS_roswifibot_VnMatrix3x3_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace roswifibot
{

  class VnMatrix3x3 : public ros::Msg
  {
    public:
      double c00;
      double c01;
      double c02;
      double c10;
      double c11;
      double c12;
      double c20;
      double c21;
      double c22;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_c00;
      u_c00.real = this->c00;
      *(outbuffer + offset + 0) = (u_c00.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_c00.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_c00.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_c00.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_c00.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_c00.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_c00.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_c00.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->c00);
      union {
        double real;
        uint64_t base;
      } u_c01;
      u_c01.real = this->c01;
      *(outbuffer + offset + 0) = (u_c01.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_c01.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_c01.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_c01.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_c01.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_c01.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_c01.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_c01.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->c01);
      union {
        double real;
        uint64_t base;
      } u_c02;
      u_c02.real = this->c02;
      *(outbuffer + offset + 0) = (u_c02.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_c02.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_c02.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_c02.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_c02.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_c02.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_c02.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_c02.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->c02);
      union {
        double real;
        uint64_t base;
      } u_c10;
      u_c10.real = this->c10;
      *(outbuffer + offset + 0) = (u_c10.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_c10.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_c10.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_c10.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_c10.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_c10.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_c10.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_c10.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->c10);
      union {
        double real;
        uint64_t base;
      } u_c11;
      u_c11.real = this->c11;
      *(outbuffer + offset + 0) = (u_c11.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_c11.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_c11.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_c11.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_c11.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_c11.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_c11.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_c11.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->c11);
      union {
        double real;
        uint64_t base;
      } u_c12;
      u_c12.real = this->c12;
      *(outbuffer + offset + 0) = (u_c12.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_c12.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_c12.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_c12.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_c12.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_c12.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_c12.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_c12.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->c12);
      union {
        double real;
        uint64_t base;
      } u_c20;
      u_c20.real = this->c20;
      *(outbuffer + offset + 0) = (u_c20.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_c20.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_c20.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_c20.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_c20.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_c20.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_c20.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_c20.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->c20);
      union {
        double real;
        uint64_t base;
      } u_c21;
      u_c21.real = this->c21;
      *(outbuffer + offset + 0) = (u_c21.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_c21.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_c21.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_c21.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_c21.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_c21.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_c21.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_c21.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->c21);
      union {
        double real;
        uint64_t base;
      } u_c22;
      u_c22.real = this->c22;
      *(outbuffer + offset + 0) = (u_c22.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_c22.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_c22.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_c22.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_c22.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_c22.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_c22.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_c22.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->c22);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_c00;
      u_c00.base = 0;
      u_c00.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_c00.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_c00.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_c00.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_c00.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_c00.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_c00.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_c00.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->c00 = u_c00.real;
      offset += sizeof(this->c00);
      union {
        double real;
        uint64_t base;
      } u_c01;
      u_c01.base = 0;
      u_c01.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_c01.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_c01.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_c01.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_c01.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_c01.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_c01.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_c01.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->c01 = u_c01.real;
      offset += sizeof(this->c01);
      union {
        double real;
        uint64_t base;
      } u_c02;
      u_c02.base = 0;
      u_c02.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_c02.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_c02.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_c02.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_c02.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_c02.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_c02.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_c02.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->c02 = u_c02.real;
      offset += sizeof(this->c02);
      union {
        double real;
        uint64_t base;
      } u_c10;
      u_c10.base = 0;
      u_c10.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_c10.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_c10.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_c10.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_c10.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_c10.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_c10.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_c10.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->c10 = u_c10.real;
      offset += sizeof(this->c10);
      union {
        double real;
        uint64_t base;
      } u_c11;
      u_c11.base = 0;
      u_c11.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_c11.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_c11.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_c11.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_c11.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_c11.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_c11.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_c11.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->c11 = u_c11.real;
      offset += sizeof(this->c11);
      union {
        double real;
        uint64_t base;
      } u_c12;
      u_c12.base = 0;
      u_c12.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_c12.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_c12.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_c12.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_c12.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_c12.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_c12.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_c12.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->c12 = u_c12.real;
      offset += sizeof(this->c12);
      union {
        double real;
        uint64_t base;
      } u_c20;
      u_c20.base = 0;
      u_c20.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_c20.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_c20.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_c20.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_c20.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_c20.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_c20.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_c20.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->c20 = u_c20.real;
      offset += sizeof(this->c20);
      union {
        double real;
        uint64_t base;
      } u_c21;
      u_c21.base = 0;
      u_c21.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_c21.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_c21.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_c21.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_c21.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_c21.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_c21.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_c21.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->c21 = u_c21.real;
      offset += sizeof(this->c21);
      union {
        double real;
        uint64_t base;
      } u_c22;
      u_c22.base = 0;
      u_c22.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_c22.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_c22.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_c22.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_c22.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_c22.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_c22.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_c22.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->c22 = u_c22.real;
      offset += sizeof(this->c22);
     return offset;
    }

    const char * getType(){ return "roswifibot/VnMatrix3x3"; };
    const char * getMD5(){ return "3fd32eb56edcd11d815b05ae3708757f"; };

  };

}
#endif