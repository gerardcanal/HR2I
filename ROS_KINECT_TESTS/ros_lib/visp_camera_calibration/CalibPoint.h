#ifndef _ROS_visp_camera_calibration_CalibPoint_h
#define _ROS_visp_camera_calibration_CalibPoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace visp_camera_calibration
{

  class CalibPoint : public ros::Msg
  {
    public:
      int32_t i;
      int32_t j;
      double X;
      double Y;
      double Z;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_i;
      u_i.real = this->i;
      *(outbuffer + offset + 0) = (u_i.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_i.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_i.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_i.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->i);
      union {
        int32_t real;
        uint32_t base;
      } u_j;
      u_j.real = this->j;
      *(outbuffer + offset + 0) = (u_j.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_j.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_j.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_j.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->j);
      union {
        double real;
        uint64_t base;
      } u_X;
      u_X.real = this->X;
      *(outbuffer + offset + 0) = (u_X.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_X.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_X.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_X.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_X.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_X.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_X.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_X.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->X);
      union {
        double real;
        uint64_t base;
      } u_Y;
      u_Y.real = this->Y;
      *(outbuffer + offset + 0) = (u_Y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Y.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_Y.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_Y.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_Y.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_Y.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->Y);
      union {
        double real;
        uint64_t base;
      } u_Z;
      u_Z.real = this->Z;
      *(outbuffer + offset + 0) = (u_Z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Z.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_Z.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_Z.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_Z.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_Z.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->Z);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_i;
      u_i.base = 0;
      u_i.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_i.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_i.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_i.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->i = u_i.real;
      offset += sizeof(this->i);
      union {
        int32_t real;
        uint32_t base;
      } u_j;
      u_j.base = 0;
      u_j.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_j.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_j.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_j.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->j = u_j.real;
      offset += sizeof(this->j);
      union {
        double real;
        uint64_t base;
      } u_X;
      u_X.base = 0;
      u_X.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_X.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_X.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_X.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_X.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_X.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_X.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_X.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->X = u_X.real;
      offset += sizeof(this->X);
      union {
        double real;
        uint64_t base;
      } u_Y;
      u_Y.base = 0;
      u_Y.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Y.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Y.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Y.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_Y.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_Y.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_Y.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_Y.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->Y = u_Y.real;
      offset += sizeof(this->Y);
      union {
        double real;
        uint64_t base;
      } u_Z;
      u_Z.base = 0;
      u_Z.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Z.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Z.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Z.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_Z.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_Z.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_Z.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_Z.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->Z = u_Z.real;
      offset += sizeof(this->Z);
     return offset;
    }

    const char * getType(){ return "visp_camera_calibration/CalibPoint"; };
    const char * getMD5(){ return "9b9d41ff2127a593c07cec2a11858ac5"; };

  };

}
#endif