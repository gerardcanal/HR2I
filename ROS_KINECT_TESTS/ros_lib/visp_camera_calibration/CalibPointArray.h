#ifndef _ROS_visp_camera_calibration_CalibPointArray_h
#define _ROS_visp_camera_calibration_CalibPointArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "visp_camera_calibration/CalibPoint.h"

namespace visp_camera_calibration
{

  class CalibPointArray : public ros::Msg
  {
    public:
      uint8_t points_length;
      visp_camera_calibration::CalibPoint st_points;
      visp_camera_calibration::CalibPoint * points;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
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
      uint8_t points_lengthT = *(inbuffer + offset++);
      if(points_lengthT > points_length)
        this->points = (visp_camera_calibration::CalibPoint*)realloc(this->points, points_lengthT * sizeof(visp_camera_calibration::CalibPoint));
      offset += 3;
      points_length = points_lengthT;
      for( uint8_t i = 0; i < points_length; i++){
      offset += this->st_points.deserialize(inbuffer + offset);
        memcpy( &(this->points[i]), &(this->st_points), sizeof(visp_camera_calibration::CalibPoint));
      }
     return offset;
    }

    const char * getType(){ return "visp_camera_calibration/CalibPointArray"; };
    const char * getMD5(){ return "46e7b53381d96d2d7cbbb7418f6dd696"; };

  };

}
#endif