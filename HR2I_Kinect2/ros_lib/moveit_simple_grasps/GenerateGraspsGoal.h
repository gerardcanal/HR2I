#ifndef _ROS_moveit_simple_grasps_GenerateGraspsGoal_h
#define _ROS_moveit_simple_grasps_GenerateGraspsGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"
#include "moveit_simple_grasps/GraspGeneratorOptions.h"

namespace moveit_simple_grasps
{

  class GenerateGraspsGoal : public ros::Msg
  {
    public:
      geometry_msgs::Pose pose;
      double width;
      uint8_t options_length;
      moveit_simple_grasps::GraspGeneratorOptions st_options;
      moveit_simple_grasps::GraspGeneratorOptions * options;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pose.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_width;
      u_width.real = this->width;
      *(outbuffer + offset + 0) = (u_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_width.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_width.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_width.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_width.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_width.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->width);
      *(outbuffer + offset++) = options_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < options_length; i++){
      offset += this->options[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pose.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_width;
      u_width.base = 0;
      u_width.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_width.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_width.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_width.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_width.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_width.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_width.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_width.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->width = u_width.real;
      offset += sizeof(this->width);
      uint8_t options_lengthT = *(inbuffer + offset++);
      if(options_lengthT > options_length)
        this->options = (moveit_simple_grasps::GraspGeneratorOptions*)realloc(this->options, options_lengthT * sizeof(moveit_simple_grasps::GraspGeneratorOptions));
      offset += 3;
      options_length = options_lengthT;
      for( uint8_t i = 0; i < options_length; i++){
      offset += this->st_options.deserialize(inbuffer + offset);
        memcpy( &(this->options[i]), &(this->st_options), sizeof(moveit_simple_grasps::GraspGeneratorOptions));
      }
     return offset;
    }

    const char * getType(){ return "moveit_simple_grasps/GenerateGraspsGoal"; };
    const char * getMD5(){ return "162875df5b62e9fb4be6d2aa5a67176d"; };

  };

}
#endif