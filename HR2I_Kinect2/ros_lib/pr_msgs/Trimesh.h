#ifndef _ROS_pr_msgs_Trimesh_h
#define _ROS_pr_msgs_Trimesh_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point32.h"

namespace pr_msgs
{

  class Trimesh : public ros::Msg
  {
    public:
      std_msgs::Header header;
      char * name;
      uint8_t vertices_length;
      geometry_msgs::Point32 st_vertices;
      geometry_msgs::Point32 * vertices;
      uint8_t vertex_indices_length;
      uint32_t st_vertex_indices;
      uint32_t * vertex_indices;
      uint8_t colors_length;
      uint32_t st_colors;
      uint32_t * colors;
      uint8_t color_indices_length;
      uint32_t st_color_indices;
      uint32_t * color_indices;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_name = strlen( (const char*) this->name);
      memcpy(outbuffer + offset, &length_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      *(outbuffer + offset++) = vertices_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < vertices_length; i++){
      offset += this->vertices[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = vertex_indices_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < vertex_indices_length; i++){
      *(outbuffer + offset + 0) = (this->vertex_indices[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->vertex_indices[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->vertex_indices[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->vertex_indices[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vertex_indices[i]);
      }
      *(outbuffer + offset++) = colors_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < colors_length; i++){
      *(outbuffer + offset + 0) = (this->colors[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->colors[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->colors[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->colors[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->colors[i]);
      }
      *(outbuffer + offset++) = color_indices_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < color_indices_length; i++){
      *(outbuffer + offset + 0) = (this->color_indices[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->color_indices[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->color_indices[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->color_indices[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->color_indices[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_name;
      memcpy(&length_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint8_t vertices_lengthT = *(inbuffer + offset++);
      if(vertices_lengthT > vertices_length)
        this->vertices = (geometry_msgs::Point32*)realloc(this->vertices, vertices_lengthT * sizeof(geometry_msgs::Point32));
      offset += 3;
      vertices_length = vertices_lengthT;
      for( uint8_t i = 0; i < vertices_length; i++){
      offset += this->st_vertices.deserialize(inbuffer + offset);
        memcpy( &(this->vertices[i]), &(this->st_vertices), sizeof(geometry_msgs::Point32));
      }
      uint8_t vertex_indices_lengthT = *(inbuffer + offset++);
      if(vertex_indices_lengthT > vertex_indices_length)
        this->vertex_indices = (uint32_t*)realloc(this->vertex_indices, vertex_indices_lengthT * sizeof(uint32_t));
      offset += 3;
      vertex_indices_length = vertex_indices_lengthT;
      for( uint8_t i = 0; i < vertex_indices_length; i++){
      this->st_vertex_indices =  ((uint32_t) (*(inbuffer + offset)));
      this->st_vertex_indices |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_vertex_indices |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_vertex_indices |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_vertex_indices);
        memcpy( &(this->vertex_indices[i]), &(this->st_vertex_indices), sizeof(uint32_t));
      }
      uint8_t colors_lengthT = *(inbuffer + offset++);
      if(colors_lengthT > colors_length)
        this->colors = (uint32_t*)realloc(this->colors, colors_lengthT * sizeof(uint32_t));
      offset += 3;
      colors_length = colors_lengthT;
      for( uint8_t i = 0; i < colors_length; i++){
      this->st_colors =  ((uint32_t) (*(inbuffer + offset)));
      this->st_colors |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_colors |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_colors |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_colors);
        memcpy( &(this->colors[i]), &(this->st_colors), sizeof(uint32_t));
      }
      uint8_t color_indices_lengthT = *(inbuffer + offset++);
      if(color_indices_lengthT > color_indices_length)
        this->color_indices = (uint32_t*)realloc(this->color_indices, color_indices_lengthT * sizeof(uint32_t));
      offset += 3;
      color_indices_length = color_indices_lengthT;
      for( uint8_t i = 0; i < color_indices_length; i++){
      this->st_color_indices =  ((uint32_t) (*(inbuffer + offset)));
      this->st_color_indices |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_color_indices |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_color_indices |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_color_indices);
        memcpy( &(this->color_indices[i]), &(this->st_color_indices), sizeof(uint32_t));
      }
     return offset;
    }

    const char * getType(){ return "pr_msgs/Trimesh"; };
    const char * getMD5(){ return "c4732217043c8a79c8374a3de7386041"; };

  };

}
#endif