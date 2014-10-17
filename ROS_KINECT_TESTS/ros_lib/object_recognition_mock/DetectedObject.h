#ifndef _ROS_object_recognition_mock_DetectedObject_h
#define _ROS_object_recognition_mock_DetectedObject_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "object_recognition_mock/Pixel.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"

namespace object_recognition_mock
{

  class DetectedObject : public ros::Msg
  {
    public:
      uint8_t points2d_length;
      object_recognition_mock::Pixel st_points2d;
      object_recognition_mock::Pixel * points2d;
      uint8_t points3d_length;
      geometry_msgs::Point st_points3d;
      geometry_msgs::Point * points3d;
      geometry_msgs::Pose pose;
      uint8_t points3d_model_length;
      geometry_msgs::Point st_points3d_model;
      geometry_msgs::Point * points3d_model;
      uint8_t octave_length;
      int32_t st_octave;
      int32_t * octave;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = points2d_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < points2d_length; i++){
      offset += this->points2d[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = points3d_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < points3d_length; i++){
      offset += this->points3d[i].serialize(outbuffer + offset);
      }
      offset += this->pose.serialize(outbuffer + offset);
      *(outbuffer + offset++) = points3d_model_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < points3d_model_length; i++){
      offset += this->points3d_model[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = octave_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < octave_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_octavei;
      u_octavei.real = this->octave[i];
      *(outbuffer + offset + 0) = (u_octavei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_octavei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_octavei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_octavei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->octave[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t points2d_lengthT = *(inbuffer + offset++);
      if(points2d_lengthT > points2d_length)
        this->points2d = (object_recognition_mock::Pixel*)realloc(this->points2d, points2d_lengthT * sizeof(object_recognition_mock::Pixel));
      offset += 3;
      points2d_length = points2d_lengthT;
      for( uint8_t i = 0; i < points2d_length; i++){
      offset += this->st_points2d.deserialize(inbuffer + offset);
        memcpy( &(this->points2d[i]), &(this->st_points2d), sizeof(object_recognition_mock::Pixel));
      }
      uint8_t points3d_lengthT = *(inbuffer + offset++);
      if(points3d_lengthT > points3d_length)
        this->points3d = (geometry_msgs::Point*)realloc(this->points3d, points3d_lengthT * sizeof(geometry_msgs::Point));
      offset += 3;
      points3d_length = points3d_lengthT;
      for( uint8_t i = 0; i < points3d_length; i++){
      offset += this->st_points3d.deserialize(inbuffer + offset);
        memcpy( &(this->points3d[i]), &(this->st_points3d), sizeof(geometry_msgs::Point));
      }
      offset += this->pose.deserialize(inbuffer + offset);
      uint8_t points3d_model_lengthT = *(inbuffer + offset++);
      if(points3d_model_lengthT > points3d_model_length)
        this->points3d_model = (geometry_msgs::Point*)realloc(this->points3d_model, points3d_model_lengthT * sizeof(geometry_msgs::Point));
      offset += 3;
      points3d_model_length = points3d_model_lengthT;
      for( uint8_t i = 0; i < points3d_model_length; i++){
      offset += this->st_points3d_model.deserialize(inbuffer + offset);
        memcpy( &(this->points3d_model[i]), &(this->st_points3d_model), sizeof(geometry_msgs::Point));
      }
      uint8_t octave_lengthT = *(inbuffer + offset++);
      if(octave_lengthT > octave_length)
        this->octave = (int32_t*)realloc(this->octave, octave_lengthT * sizeof(int32_t));
      offset += 3;
      octave_length = octave_lengthT;
      for( uint8_t i = 0; i < octave_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_octave;
      u_st_octave.base = 0;
      u_st_octave.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_octave.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_octave.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_octave.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_octave = u_st_octave.real;
      offset += sizeof(this->st_octave);
        memcpy( &(this->octave[i]), &(this->st_octave), sizeof(int32_t));
      }
     return offset;
    }

    const char * getType(){ return "object_recognition_mock/DetectedObject"; };
    const char * getMD5(){ return "db086185ac6f68f8b4560ea38a8ec39b"; };

  };

}
#endif