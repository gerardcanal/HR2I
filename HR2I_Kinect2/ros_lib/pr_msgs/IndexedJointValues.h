#ifndef _ROS_pr_msgs_IndexedJointValues_h
#define _ROS_pr_msgs_IndexedJointValues_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

  class IndexedJointValues : public ros::Msg
  {
    public:
      uint8_t jointIndices_length;
      int32_t st_jointIndices;
      int32_t * jointIndices;
      uint8_t values_length;
      double st_values;
      double * values;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = jointIndices_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < jointIndices_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_jointIndicesi;
      u_jointIndicesi.real = this->jointIndices[i];
      *(outbuffer + offset + 0) = (u_jointIndicesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_jointIndicesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_jointIndicesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_jointIndicesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->jointIndices[i]);
      }
      *(outbuffer + offset++) = values_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < values_length; i++){
      union {
        double real;
        uint64_t base;
      } u_valuesi;
      u_valuesi.real = this->values[i];
      *(outbuffer + offset + 0) = (u_valuesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_valuesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_valuesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_valuesi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_valuesi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_valuesi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_valuesi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_valuesi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->values[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t jointIndices_lengthT = *(inbuffer + offset++);
      if(jointIndices_lengthT > jointIndices_length)
        this->jointIndices = (int32_t*)realloc(this->jointIndices, jointIndices_lengthT * sizeof(int32_t));
      offset += 3;
      jointIndices_length = jointIndices_lengthT;
      for( uint8_t i = 0; i < jointIndices_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_jointIndices;
      u_st_jointIndices.base = 0;
      u_st_jointIndices.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_jointIndices.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_jointIndices.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_jointIndices.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_jointIndices = u_st_jointIndices.real;
      offset += sizeof(this->st_jointIndices);
        memcpy( &(this->jointIndices[i]), &(this->st_jointIndices), sizeof(int32_t));
      }
      uint8_t values_lengthT = *(inbuffer + offset++);
      if(values_lengthT > values_length)
        this->values = (double*)realloc(this->values, values_lengthT * sizeof(double));
      offset += 3;
      values_length = values_lengthT;
      for( uint8_t i = 0; i < values_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_values;
      u_st_values.base = 0;
      u_st_values.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_values.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_values.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_values.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_values.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_values.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_values.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_values.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_values = u_st_values.real;
      offset += sizeof(this->st_values);
        memcpy( &(this->values[i]), &(this->st_values), sizeof(double));
      }
     return offset;
    }

    const char * getType(){ return "pr_msgs/IndexedJointValues"; };
    const char * getMD5(){ return "02a472aa60e521a148bf4000eda4e325"; };

  };

}
#endif