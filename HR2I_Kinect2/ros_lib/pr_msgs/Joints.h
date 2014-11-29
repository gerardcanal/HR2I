#ifndef _ROS_pr_msgs_Joints_h
#define _ROS_pr_msgs_Joints_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

  class Joints : public ros::Msg
  {
    public:
      uint8_t j_length;
      double st_j;
      double * j;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = j_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < j_length; i++){
      union {
        double real;
        uint64_t base;
      } u_ji;
      u_ji.real = this->j[i];
      *(outbuffer + offset + 0) = (u_ji.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ji.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ji.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ji.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_ji.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_ji.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_ji.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_ji.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->j[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t j_lengthT = *(inbuffer + offset++);
      if(j_lengthT > j_length)
        this->j = (double*)realloc(this->j, j_lengthT * sizeof(double));
      offset += 3;
      j_length = j_lengthT;
      for( uint8_t i = 0; i < j_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_j;
      u_st_j.base = 0;
      u_st_j.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_j.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_j.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_j.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_j.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_j.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_j.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_j.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_j = u_st_j.real;
      offset += sizeof(this->st_j);
        memcpy( &(this->j[i]), &(this->st_j), sizeof(double));
      }
     return offset;
    }

    const char * getType(){ return "pr_msgs/Joints"; };
    const char * getMD5(){ return "59542e81b1fd2eaee58892b9055022e8"; };

  };

}
#endif