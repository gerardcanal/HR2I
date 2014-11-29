#ifndef _ROS_pr_msgs_MaglevSense_h
#define _ROS_pr_msgs_MaglevSense_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

  class MaglevSense : public ros::Msg
  {
    public:
      uint8_t position_length;
      float st_position;
      float * position;
      uint8_t force_length;
      float st_force;
      float * force;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = position_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < position_length; i++){
      union {
        float real;
        uint32_t base;
      } u_positioni;
      u_positioni.real = this->position[i];
      *(outbuffer + offset + 0) = (u_positioni.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_positioni.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_positioni.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_positioni.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position[i]);
      }
      *(outbuffer + offset++) = force_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < force_length; i++){
      union {
        float real;
        uint32_t base;
      } u_forcei;
      u_forcei.real = this->force[i];
      *(outbuffer + offset + 0) = (u_forcei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_forcei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_forcei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_forcei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->force[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t position_lengthT = *(inbuffer + offset++);
      if(position_lengthT > position_length)
        this->position = (float*)realloc(this->position, position_lengthT * sizeof(float));
      offset += 3;
      position_length = position_lengthT;
      for( uint8_t i = 0; i < position_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_position;
      u_st_position.base = 0;
      u_st_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_position = u_st_position.real;
      offset += sizeof(this->st_position);
        memcpy( &(this->position[i]), &(this->st_position), sizeof(float));
      }
      uint8_t force_lengthT = *(inbuffer + offset++);
      if(force_lengthT > force_length)
        this->force = (float*)realloc(this->force, force_lengthT * sizeof(float));
      offset += 3;
      force_length = force_lengthT;
      for( uint8_t i = 0; i < force_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_force;
      u_st_force.base = 0;
      u_st_force.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_force.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_force.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_force.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_force = u_st_force.real;
      offset += sizeof(this->st_force);
        memcpy( &(this->force[i]), &(this->st_force), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "pr_msgs/MaglevSense"; };
    const char * getMD5(){ return "82871840a2fd06a041e9e9618073ff7c"; };

  };

}
#endif