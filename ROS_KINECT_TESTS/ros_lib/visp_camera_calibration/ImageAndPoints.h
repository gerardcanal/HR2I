#ifndef _ROS_visp_camera_calibration_ImageAndPoints_h
#define _ROS_visp_camera_calibration_ImageAndPoints_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include "visp_camera_calibration/ImagePoint.h"

namespace visp_camera_calibration
{

  class ImageAndPoints : public ros::Msg
  {
    public:
      std_msgs::Header header;
      sensor_msgs::Image image;
      uint8_t points_length;
      visp_camera_calibration::ImagePoint st_points;
      visp_camera_calibration::ImagePoint * points;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->image.serialize(outbuffer + offset);
      *(outbuffer + offset++) = points_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < points_length; i++){
      offset += this->points[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->image.deserialize(inbuffer + offset);
      uint8_t points_lengthT = *(inbuffer + offset++);
      if(points_lengthT > points_length)
        this->points = (visp_camera_calibration::ImagePoint*)realloc(this->points, points_lengthT * sizeof(visp_camera_calibration::ImagePoint));
      offset += 3;
      points_length = points_lengthT;
      for( uint8_t i = 0; i < points_length; i++){
      offset += this->st_points.deserialize(inbuffer + offset);
        memcpy( &(this->points[i]), &(this->st_points), sizeof(visp_camera_calibration::ImagePoint));
      }
     return offset;
    }

    const char * getType(){ return "visp_camera_calibration/ImageAndPoints"; };
    const char * getMD5(){ return "0fea5fc0844344ec4ec1adadebd18f75"; };

  };

}
#endif