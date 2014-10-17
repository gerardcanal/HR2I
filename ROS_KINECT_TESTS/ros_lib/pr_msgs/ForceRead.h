#ifndef _ROS_pr_msgs_ForceRead_h
#define _ROS_pr_msgs_ForceRead_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

  class ForceRead : public ros::Msg
  {
    public:
      uint8_t force_length;
      double st_force;
      double * force;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = force_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < force_length; i++){
      union {
        double real;
        uint64_t base;
      } u_forcei;
      u_forcei.real = this->force[i];
      *(outbuffer + offset + 0) = (u_forcei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_forcei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_forcei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_forcei.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_forcei.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_forcei.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_forcei.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_forcei.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->force[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t force_lengthT = *(inbuffer + offset++);
      if(force_lengthT > force_length)
        this->force = (double*)realloc(this->force, force_lengthT * sizeof(double));
      offset += 3;
      force_length = force_lengthT;
      for( uint8_t i = 0; i < force_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_force;
      u_st_force.base = 0;
      u_st_force.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_force.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_force.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_force.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_force.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_force.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_force.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_force.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_force = u_st_force.real;
      offset += sizeof(this->st_force);
        memcpy( &(this->force[i]), &(this->st_force), sizeof(double));
      }
     return offset;
    }

    const char * getType(){ return "pr_msgs/ForceRead"; };
    const char * getMD5(){ return "23399487a2048efabaa375690609f3b9"; };

  };

}
#endif