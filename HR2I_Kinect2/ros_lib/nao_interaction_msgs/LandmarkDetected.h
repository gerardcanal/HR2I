#ifndef _ROS_nao_interaction_msgs_LandmarkDetected_h
#define _ROS_nao_interaction_msgs_LandmarkDetected_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/String.h"

namespace nao_interaction_msgs
{

  class LandmarkDetected : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t mark_ids_length;
      std_msgs::Int32 st_mark_ids;
      std_msgs::Int32 * mark_ids;
      uint8_t shape_alpha_length;
      std_msgs::Float32 st_shape_alpha;
      std_msgs::Float32 * shape_alpha;
      uint8_t shape_beta_length;
      std_msgs::Float32 st_shape_beta;
      std_msgs::Float32 * shape_beta;
      uint8_t shape_sizex_length;
      std_msgs::Float32 st_shape_sizex;
      std_msgs::Float32 * shape_sizex;
      uint8_t shape_sizey_length;
      std_msgs::Float32 st_shape_sizey;
      std_msgs::Float32 * shape_sizey;
      geometry_msgs::Pose camera_local_pose;
      geometry_msgs::Pose camera_world_pose;
      std_msgs::String camera_name;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = mark_ids_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < mark_ids_length; i++){
      offset += this->mark_ids[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = shape_alpha_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < shape_alpha_length; i++){
      offset += this->shape_alpha[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = shape_beta_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < shape_beta_length; i++){
      offset += this->shape_beta[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = shape_sizex_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < shape_sizex_length; i++){
      offset += this->shape_sizex[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = shape_sizey_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < shape_sizey_length; i++){
      offset += this->shape_sizey[i].serialize(outbuffer + offset);
      }
      offset += this->camera_local_pose.serialize(outbuffer + offset);
      offset += this->camera_world_pose.serialize(outbuffer + offset);
      offset += this->camera_name.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t mark_ids_lengthT = *(inbuffer + offset++);
      if(mark_ids_lengthT > mark_ids_length)
        this->mark_ids = (std_msgs::Int32*)realloc(this->mark_ids, mark_ids_lengthT * sizeof(std_msgs::Int32));
      offset += 3;
      mark_ids_length = mark_ids_lengthT;
      for( uint8_t i = 0; i < mark_ids_length; i++){
      offset += this->st_mark_ids.deserialize(inbuffer + offset);
        memcpy( &(this->mark_ids[i]), &(this->st_mark_ids), sizeof(std_msgs::Int32));
      }
      uint8_t shape_alpha_lengthT = *(inbuffer + offset++);
      if(shape_alpha_lengthT > shape_alpha_length)
        this->shape_alpha = (std_msgs::Float32*)realloc(this->shape_alpha, shape_alpha_lengthT * sizeof(std_msgs::Float32));
      offset += 3;
      shape_alpha_length = shape_alpha_lengthT;
      for( uint8_t i = 0; i < shape_alpha_length; i++){
      offset += this->st_shape_alpha.deserialize(inbuffer + offset);
        memcpy( &(this->shape_alpha[i]), &(this->st_shape_alpha), sizeof(std_msgs::Float32));
      }
      uint8_t shape_beta_lengthT = *(inbuffer + offset++);
      if(shape_beta_lengthT > shape_beta_length)
        this->shape_beta = (std_msgs::Float32*)realloc(this->shape_beta, shape_beta_lengthT * sizeof(std_msgs::Float32));
      offset += 3;
      shape_beta_length = shape_beta_lengthT;
      for( uint8_t i = 0; i < shape_beta_length; i++){
      offset += this->st_shape_beta.deserialize(inbuffer + offset);
        memcpy( &(this->shape_beta[i]), &(this->st_shape_beta), sizeof(std_msgs::Float32));
      }
      uint8_t shape_sizex_lengthT = *(inbuffer + offset++);
      if(shape_sizex_lengthT > shape_sizex_length)
        this->shape_sizex = (std_msgs::Float32*)realloc(this->shape_sizex, shape_sizex_lengthT * sizeof(std_msgs::Float32));
      offset += 3;
      shape_sizex_length = shape_sizex_lengthT;
      for( uint8_t i = 0; i < shape_sizex_length; i++){
      offset += this->st_shape_sizex.deserialize(inbuffer + offset);
        memcpy( &(this->shape_sizex[i]), &(this->st_shape_sizex), sizeof(std_msgs::Float32));
      }
      uint8_t shape_sizey_lengthT = *(inbuffer + offset++);
      if(shape_sizey_lengthT > shape_sizey_length)
        this->shape_sizey = (std_msgs::Float32*)realloc(this->shape_sizey, shape_sizey_lengthT * sizeof(std_msgs::Float32));
      offset += 3;
      shape_sizey_length = shape_sizey_lengthT;
      for( uint8_t i = 0; i < shape_sizey_length; i++){
      offset += this->st_shape_sizey.deserialize(inbuffer + offset);
        memcpy( &(this->shape_sizey[i]), &(this->st_shape_sizey), sizeof(std_msgs::Float32));
      }
      offset += this->camera_local_pose.deserialize(inbuffer + offset);
      offset += this->camera_world_pose.deserialize(inbuffer + offset);
      offset += this->camera_name.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "nao_interaction_msgs/LandmarkDetected"; };
    const char * getMD5(){ return "4d97e85c0a306501da5d22b795cdac76"; };

  };

}
#endif