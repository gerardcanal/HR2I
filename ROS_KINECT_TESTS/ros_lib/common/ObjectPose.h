#ifndef _ROS_common_ObjectPose_h
#define _ROS_common_ObjectPose_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point32.h"
#include "common/NameTypeValue.h"

namespace common
{

  class ObjectPose : public ros::Msg
  {
    public:
      char * name;
      geometry_msgs::Pose pose;
      geometry_msgs::Point32 pose2D;
      uint8_t convex_hull_x_length;
      int16_t st_convex_hull_x;
      int16_t * convex_hull_x;
      uint8_t convex_hull_y_length;
      int16_t st_convex_hull_y;
      int16_t * convex_hull_y;
      float mean_quality;
      int16_t used_points;
      uint8_t properties_length;
      common::NameTypeValue st_properties;
      common::NameTypeValue * properties;
      uint8_t pose_uncertainty_list_length;
      geometry_msgs::Pose st_pose_uncertainty_list;
      geometry_msgs::Pose * pose_uncertainty_list;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen( (const char*) this->name);
      memcpy(outbuffer + offset, &length_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->pose2D.serialize(outbuffer + offset);
      *(outbuffer + offset++) = convex_hull_x_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < convex_hull_x_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_convex_hull_xi;
      u_convex_hull_xi.real = this->convex_hull_x[i];
      *(outbuffer + offset + 0) = (u_convex_hull_xi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_convex_hull_xi.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->convex_hull_x[i]);
      }
      *(outbuffer + offset++) = convex_hull_y_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < convex_hull_y_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_convex_hull_yi;
      u_convex_hull_yi.real = this->convex_hull_y[i];
      *(outbuffer + offset + 0) = (u_convex_hull_yi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_convex_hull_yi.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->convex_hull_y[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_mean_quality;
      u_mean_quality.real = this->mean_quality;
      *(outbuffer + offset + 0) = (u_mean_quality.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mean_quality.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mean_quality.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mean_quality.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mean_quality);
      union {
        int16_t real;
        uint16_t base;
      } u_used_points;
      u_used_points.real = this->used_points;
      *(outbuffer + offset + 0) = (u_used_points.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_used_points.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->used_points);
      *(outbuffer + offset++) = properties_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < properties_length; i++){
      offset += this->properties[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = pose_uncertainty_list_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < pose_uncertainty_list_length; i++){
      offset += this->pose_uncertainty_list[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      memcpy(&length_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->pose2D.deserialize(inbuffer + offset);
      uint8_t convex_hull_x_lengthT = *(inbuffer + offset++);
      if(convex_hull_x_lengthT > convex_hull_x_length)
        this->convex_hull_x = (int16_t*)realloc(this->convex_hull_x, convex_hull_x_lengthT * sizeof(int16_t));
      offset += 3;
      convex_hull_x_length = convex_hull_x_lengthT;
      for( uint8_t i = 0; i < convex_hull_x_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_st_convex_hull_x;
      u_st_convex_hull_x.base = 0;
      u_st_convex_hull_x.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_convex_hull_x.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_convex_hull_x = u_st_convex_hull_x.real;
      offset += sizeof(this->st_convex_hull_x);
        memcpy( &(this->convex_hull_x[i]), &(this->st_convex_hull_x), sizeof(int16_t));
      }
      uint8_t convex_hull_y_lengthT = *(inbuffer + offset++);
      if(convex_hull_y_lengthT > convex_hull_y_length)
        this->convex_hull_y = (int16_t*)realloc(this->convex_hull_y, convex_hull_y_lengthT * sizeof(int16_t));
      offset += 3;
      convex_hull_y_length = convex_hull_y_lengthT;
      for( uint8_t i = 0; i < convex_hull_y_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_st_convex_hull_y;
      u_st_convex_hull_y.base = 0;
      u_st_convex_hull_y.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_convex_hull_y.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_convex_hull_y = u_st_convex_hull_y.real;
      offset += sizeof(this->st_convex_hull_y);
        memcpy( &(this->convex_hull_y[i]), &(this->st_convex_hull_y), sizeof(int16_t));
      }
      union {
        float real;
        uint32_t base;
      } u_mean_quality;
      u_mean_quality.base = 0;
      u_mean_quality.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mean_quality.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mean_quality.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mean_quality.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mean_quality = u_mean_quality.real;
      offset += sizeof(this->mean_quality);
      union {
        int16_t real;
        uint16_t base;
      } u_used_points;
      u_used_points.base = 0;
      u_used_points.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_used_points.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->used_points = u_used_points.real;
      offset += sizeof(this->used_points);
      uint8_t properties_lengthT = *(inbuffer + offset++);
      if(properties_lengthT > properties_length)
        this->properties = (common::NameTypeValue*)realloc(this->properties, properties_lengthT * sizeof(common::NameTypeValue));
      offset += 3;
      properties_length = properties_lengthT;
      for( uint8_t i = 0; i < properties_length; i++){
      offset += this->st_properties.deserialize(inbuffer + offset);
        memcpy( &(this->properties[i]), &(this->st_properties), sizeof(common::NameTypeValue));
      }
      uint8_t pose_uncertainty_list_lengthT = *(inbuffer + offset++);
      if(pose_uncertainty_list_lengthT > pose_uncertainty_list_length)
        this->pose_uncertainty_list = (geometry_msgs::Pose*)realloc(this->pose_uncertainty_list, pose_uncertainty_list_lengthT * sizeof(geometry_msgs::Pose));
      offset += 3;
      pose_uncertainty_list_length = pose_uncertainty_list_lengthT;
      for( uint8_t i = 0; i < pose_uncertainty_list_length; i++){
      offset += this->st_pose_uncertainty_list.deserialize(inbuffer + offset);
        memcpy( &(this->pose_uncertainty_list[i]), &(this->st_pose_uncertainty_list), sizeof(geometry_msgs::Pose));
      }
     return offset;
    }

    const char * getType(){ return "common/ObjectPose"; };
    const char * getMD5(){ return "a1a9ddd9cdd7cb9b07994b75cc08bdcc"; };

  };

}
#endif