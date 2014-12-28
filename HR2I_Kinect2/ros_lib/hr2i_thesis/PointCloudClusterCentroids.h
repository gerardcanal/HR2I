#ifndef _ROS_hr2i_thesis_PointCloudClusterCentroids_h
#define _ROS_hr2i_thesis_PointCloudClusterCentroids_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"

namespace hr2i_thesis
{

  class PointCloudClusterCentroids : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t cluster_centroids_length;
      geometry_msgs::Point st_cluster_centroids;
      geometry_msgs::Point * cluster_centroids;
      uint8_t cluster_sizes_length;
      float st_cluster_sizes;
      float * cluster_sizes;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = cluster_centroids_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < cluster_centroids_length; i++){
      offset += this->cluster_centroids[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = cluster_sizes_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < cluster_sizes_length; i++){
      union {
        float real;
        uint32_t base;
      } u_cluster_sizesi;
      u_cluster_sizesi.real = this->cluster_sizes[i];
      *(outbuffer + offset + 0) = (u_cluster_sizesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cluster_sizesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cluster_sizesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cluster_sizesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cluster_sizes[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t cluster_centroids_lengthT = *(inbuffer + offset++);
      if(cluster_centroids_lengthT > cluster_centroids_length)
        this->cluster_centroids = (geometry_msgs::Point*)realloc(this->cluster_centroids, cluster_centroids_lengthT * sizeof(geometry_msgs::Point));
      offset += 3;
      cluster_centroids_length = cluster_centroids_lengthT;
      for( uint8_t i = 0; i < cluster_centroids_length; i++){
      offset += this->st_cluster_centroids.deserialize(inbuffer + offset);
        memcpy( &(this->cluster_centroids[i]), &(this->st_cluster_centroids), sizeof(geometry_msgs::Point));
      }
      uint8_t cluster_sizes_lengthT = *(inbuffer + offset++);
      if(cluster_sizes_lengthT > cluster_sizes_length)
        this->cluster_sizes = (float*)realloc(this->cluster_sizes, cluster_sizes_lengthT * sizeof(float));
      offset += 3;
      cluster_sizes_length = cluster_sizes_lengthT;
      for( uint8_t i = 0; i < cluster_sizes_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_cluster_sizes;
      u_st_cluster_sizes.base = 0;
      u_st_cluster_sizes.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_cluster_sizes.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_cluster_sizes.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_cluster_sizes.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_cluster_sizes = u_st_cluster_sizes.real;
      offset += sizeof(this->st_cluster_sizes);
        memcpy( &(this->cluster_sizes[i]), &(this->st_cluster_sizes), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "hr2i_thesis/PointCloudClusterCentroids"; };
    const char * getMD5(){ return "f059ae532bec9197ac74a27ce137ec14"; };

  };

}
#endif