#ifndef _ROS_pr_msgs_BHTactile_h
#define _ROS_pr_msgs_BHTactile_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

  class BHTactile : public ros::Msg
  {
    public:
      uint8_t finger1_length;
      float st_finger1;
      float * finger1;
      uint8_t finger2_length;
      float st_finger2;
      float * finger2;
      uint8_t finger3_length;
      float st_finger3;
      float * finger3;
      uint8_t palm_length;
      float st_palm;
      float * palm;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = finger1_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < finger1_length; i++){
      union {
        float real;
        uint32_t base;
      } u_finger1i;
      u_finger1i.real = this->finger1[i];
      *(outbuffer + offset + 0) = (u_finger1i.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_finger1i.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_finger1i.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_finger1i.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->finger1[i]);
      }
      *(outbuffer + offset++) = finger2_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < finger2_length; i++){
      union {
        float real;
        uint32_t base;
      } u_finger2i;
      u_finger2i.real = this->finger2[i];
      *(outbuffer + offset + 0) = (u_finger2i.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_finger2i.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_finger2i.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_finger2i.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->finger2[i]);
      }
      *(outbuffer + offset++) = finger3_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < finger3_length; i++){
      union {
        float real;
        uint32_t base;
      } u_finger3i;
      u_finger3i.real = this->finger3[i];
      *(outbuffer + offset + 0) = (u_finger3i.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_finger3i.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_finger3i.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_finger3i.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->finger3[i]);
      }
      *(outbuffer + offset++) = palm_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < palm_length; i++){
      union {
        float real;
        uint32_t base;
      } u_palmi;
      u_palmi.real = this->palm[i];
      *(outbuffer + offset + 0) = (u_palmi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_palmi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_palmi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_palmi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->palm[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t finger1_lengthT = *(inbuffer + offset++);
      if(finger1_lengthT > finger1_length)
        this->finger1 = (float*)realloc(this->finger1, finger1_lengthT * sizeof(float));
      offset += 3;
      finger1_length = finger1_lengthT;
      for( uint8_t i = 0; i < finger1_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_finger1;
      u_st_finger1.base = 0;
      u_st_finger1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_finger1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_finger1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_finger1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_finger1 = u_st_finger1.real;
      offset += sizeof(this->st_finger1);
        memcpy( &(this->finger1[i]), &(this->st_finger1), sizeof(float));
      }
      uint8_t finger2_lengthT = *(inbuffer + offset++);
      if(finger2_lengthT > finger2_length)
        this->finger2 = (float*)realloc(this->finger2, finger2_lengthT * sizeof(float));
      offset += 3;
      finger2_length = finger2_lengthT;
      for( uint8_t i = 0; i < finger2_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_finger2;
      u_st_finger2.base = 0;
      u_st_finger2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_finger2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_finger2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_finger2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_finger2 = u_st_finger2.real;
      offset += sizeof(this->st_finger2);
        memcpy( &(this->finger2[i]), &(this->st_finger2), sizeof(float));
      }
      uint8_t finger3_lengthT = *(inbuffer + offset++);
      if(finger3_lengthT > finger3_length)
        this->finger3 = (float*)realloc(this->finger3, finger3_lengthT * sizeof(float));
      offset += 3;
      finger3_length = finger3_lengthT;
      for( uint8_t i = 0; i < finger3_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_finger3;
      u_st_finger3.base = 0;
      u_st_finger3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_finger3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_finger3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_finger3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_finger3 = u_st_finger3.real;
      offset += sizeof(this->st_finger3);
        memcpy( &(this->finger3[i]), &(this->st_finger3), sizeof(float));
      }
      uint8_t palm_lengthT = *(inbuffer + offset++);
      if(palm_lengthT > palm_length)
        this->palm = (float*)realloc(this->palm, palm_lengthT * sizeof(float));
      offset += 3;
      palm_length = palm_lengthT;
      for( uint8_t i = 0; i < palm_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_palm;
      u_st_palm.base = 0;
      u_st_palm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_palm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_palm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_palm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_palm = u_st_palm.real;
      offset += sizeof(this->st_palm);
        memcpy( &(this->palm[i]), &(this->st_palm), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "pr_msgs/BHTactile"; };
    const char * getMD5(){ return "f40a82aa99391835587004e5c16d74c4"; };

  };

}
#endif