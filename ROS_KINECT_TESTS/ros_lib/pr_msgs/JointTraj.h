#ifndef _ROS_pr_msgs_JointTraj_h
#define _ROS_pr_msgs_JointTraj_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr_msgs/Joints.h"

namespace pr_msgs
{

  class JointTraj : public ros::Msg
  {
    public:
      uint8_t positions_length;
      pr_msgs::Joints st_positions;
      pr_msgs::Joints * positions;
      uint8_t blend_radius_length;
      float st_blend_radius;
      float * blend_radius;
      uint32_t options;
      enum { opt_WaitForStart = 1 };
      enum { opt_CancelOnStall = 2 };
      enum { opt_CancelOnForceInput = 4 };
      enum { opt_CancelOnTactileInput = 8 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = positions_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < positions_length; i++){
      offset += this->positions[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = blend_radius_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < blend_radius_length; i++){
      union {
        float real;
        uint32_t base;
      } u_blend_radiusi;
      u_blend_radiusi.real = this->blend_radius[i];
      *(outbuffer + offset + 0) = (u_blend_radiusi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_blend_radiusi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_blend_radiusi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_blend_radiusi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->blend_radius[i]);
      }
      *(outbuffer + offset + 0) = (this->options >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->options >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->options >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->options >> (8 * 3)) & 0xFF;
      offset += sizeof(this->options);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t positions_lengthT = *(inbuffer + offset++);
      if(positions_lengthT > positions_length)
        this->positions = (pr_msgs::Joints*)realloc(this->positions, positions_lengthT * sizeof(pr_msgs::Joints));
      offset += 3;
      positions_length = positions_lengthT;
      for( uint8_t i = 0; i < positions_length; i++){
      offset += this->st_positions.deserialize(inbuffer + offset);
        memcpy( &(this->positions[i]), &(this->st_positions), sizeof(pr_msgs::Joints));
      }
      uint8_t blend_radius_lengthT = *(inbuffer + offset++);
      if(blend_radius_lengthT > blend_radius_length)
        this->blend_radius = (float*)realloc(this->blend_radius, blend_radius_lengthT * sizeof(float));
      offset += 3;
      blend_radius_length = blend_radius_lengthT;
      for( uint8_t i = 0; i < blend_radius_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_blend_radius;
      u_st_blend_radius.base = 0;
      u_st_blend_radius.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_blend_radius.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_blend_radius.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_blend_radius.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_blend_radius = u_st_blend_radius.real;
      offset += sizeof(this->st_blend_radius);
        memcpy( &(this->blend_radius[i]), &(this->st_blend_radius), sizeof(float));
      }
      this->options =  ((uint32_t) (*(inbuffer + offset)));
      this->options |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->options |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->options |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->options);
     return offset;
    }

    const char * getType(){ return "pr_msgs/JointTraj"; };
    const char * getMD5(){ return "e07c641f5910e182dc37fb7a39f1367d"; };

  };

}
#endif