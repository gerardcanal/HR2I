#ifndef _ROS_SERVICE_calibrate_h
#define _ROS_SERVICE_calibrate_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace visp_camera_calibration
{

static const char CALIBRATE[] = "visp_camera_calibration/calibrate";

  class calibrateRequest : public ros::Msg
  {
    public:
      int32_t method;
      int32_t sample_width;
      int32_t sample_height;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_method;
      u_method.real = this->method;
      *(outbuffer + offset + 0) = (u_method.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_method.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_method.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_method.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->method);
      union {
        int32_t real;
        uint32_t base;
      } u_sample_width;
      u_sample_width.real = this->sample_width;
      *(outbuffer + offset + 0) = (u_sample_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sample_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sample_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sample_width.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sample_width);
      union {
        int32_t real;
        uint32_t base;
      } u_sample_height;
      u_sample_height.real = this->sample_height;
      *(outbuffer + offset + 0) = (u_sample_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sample_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sample_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sample_height.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sample_height);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_method;
      u_method.base = 0;
      u_method.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_method.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_method.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_method.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->method = u_method.real;
      offset += sizeof(this->method);
      union {
        int32_t real;
        uint32_t base;
      } u_sample_width;
      u_sample_width.base = 0;
      u_sample_width.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sample_width.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sample_width.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sample_width.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sample_width = u_sample_width.real;
      offset += sizeof(this->sample_width);
      union {
        int32_t real;
        uint32_t base;
      } u_sample_height;
      u_sample_height.base = 0;
      u_sample_height.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sample_height.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sample_height.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sample_height.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sample_height = u_sample_height.real;
      offset += sizeof(this->sample_height);
     return offset;
    }

    const char * getType(){ return CALIBRATE; };
    const char * getMD5(){ return "7b57c459896a8f1f8df45a385acfc123"; };

  };

  class calibrateResponse : public ros::Msg
  {
    public:
      uint8_t stdDevErrs_length;
      double st_stdDevErrs;
      double * stdDevErrs;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = stdDevErrs_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < stdDevErrs_length; i++){
      union {
        double real;
        uint64_t base;
      } u_stdDevErrsi;
      u_stdDevErrsi.real = this->stdDevErrs[i];
      *(outbuffer + offset + 0) = (u_stdDevErrsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_stdDevErrsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_stdDevErrsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_stdDevErrsi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_stdDevErrsi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_stdDevErrsi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_stdDevErrsi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_stdDevErrsi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->stdDevErrs[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t stdDevErrs_lengthT = *(inbuffer + offset++);
      if(stdDevErrs_lengthT > stdDevErrs_length)
        this->stdDevErrs = (double*)realloc(this->stdDevErrs, stdDevErrs_lengthT * sizeof(double));
      offset += 3;
      stdDevErrs_length = stdDevErrs_lengthT;
      for( uint8_t i = 0; i < stdDevErrs_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_stdDevErrs;
      u_st_stdDevErrs.base = 0;
      u_st_stdDevErrs.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_stdDevErrs.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_stdDevErrs.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_stdDevErrs.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_stdDevErrs.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_stdDevErrs.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_stdDevErrs.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_stdDevErrs.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_stdDevErrs = u_st_stdDevErrs.real;
      offset += sizeof(this->st_stdDevErrs);
        memcpy( &(this->stdDevErrs[i]), &(this->st_stdDevErrs), sizeof(double));
      }
     return offset;
    }

    const char * getType(){ return CALIBRATE; };
    const char * getMD5(){ return "cd6d27af348dbd9b7530b010497f18b0"; };

  };

  class calibrate {
    public:
    typedef calibrateRequest Request;
    typedef calibrateResponse Response;
  };

}
#endif
