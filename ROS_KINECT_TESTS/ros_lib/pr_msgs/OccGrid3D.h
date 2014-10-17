#ifndef _ROS_pr_msgs_OccGrid3D_h
#define _ROS_pr_msgs_OccGrid3D_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point32.h"

namespace pr_msgs
{

  class OccGrid3D : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float resolution;
      uint32_t num_x_voxels;
      uint32_t num_y_voxels;
      uint32_t num_z_voxels;
      geometry_msgs::Point32 origin;
      uint8_t data_length;
      int8_t st_data;
      int8_t * data;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_resolution;
      u_resolution.real = this->resolution;
      *(outbuffer + offset + 0) = (u_resolution.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_resolution.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_resolution.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_resolution.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->resolution);
      *(outbuffer + offset + 0) = (this->num_x_voxels >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->num_x_voxels >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->num_x_voxels >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->num_x_voxels >> (8 * 3)) & 0xFF;
      offset += sizeof(this->num_x_voxels);
      *(outbuffer + offset + 0) = (this->num_y_voxels >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->num_y_voxels >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->num_y_voxels >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->num_y_voxels >> (8 * 3)) & 0xFF;
      offset += sizeof(this->num_y_voxels);
      *(outbuffer + offset + 0) = (this->num_z_voxels >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->num_z_voxels >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->num_z_voxels >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->num_z_voxels >> (8 * 3)) & 0xFF;
      offset += sizeof(this->num_z_voxels);
      offset += this->origin.serialize(outbuffer + offset);
      *(outbuffer + offset++) = data_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < data_length; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_resolution;
      u_resolution.base = 0;
      u_resolution.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_resolution.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_resolution.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_resolution.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->resolution = u_resolution.real;
      offset += sizeof(this->resolution);
      this->num_x_voxels =  ((uint32_t) (*(inbuffer + offset)));
      this->num_x_voxels |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->num_x_voxels |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->num_x_voxels |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->num_x_voxels);
      this->num_y_voxels =  ((uint32_t) (*(inbuffer + offset)));
      this->num_y_voxels |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->num_y_voxels |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->num_y_voxels |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->num_y_voxels);
      this->num_z_voxels =  ((uint32_t) (*(inbuffer + offset)));
      this->num_z_voxels |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->num_z_voxels |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->num_z_voxels |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->num_z_voxels);
      offset += this->origin.deserialize(inbuffer + offset);
      uint8_t data_lengthT = *(inbuffer + offset++);
      if(data_lengthT > data_length)
        this->data = (int8_t*)realloc(this->data, data_lengthT * sizeof(int8_t));
      offset += 3;
      data_length = data_lengthT;
      for( uint8_t i = 0; i < data_length; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_st_data;
      u_st_data.base = 0;
      u_st_data.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_data = u_st_data.real;
      offset += sizeof(this->st_data);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(int8_t));
      }
     return offset;
    }

    const char * getType(){ return "pr_msgs/OccGrid3D"; };
    const char * getMD5(){ return "3c71e0896dd3dc42f5341886fcc48fd1"; };

  };

}
#endif