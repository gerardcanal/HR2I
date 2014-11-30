#ifndef _ROS_SERVICE_compute_effector_camera_h
#define _ROS_SERVICE_compute_effector_camera_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Transform.h"

namespace visp_hand2eye_calibration
{

static const char COMPUTE_EFFECTOR_CAMERA[] = "visp_hand2eye_calibration/compute_effector_camera";

  class compute_effector_cameraRequest : public ros::Msg
  {
    public:

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return COMPUTE_EFFECTOR_CAMERA; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class compute_effector_cameraResponse : public ros::Msg
  {
    public:
      geometry_msgs::Transform effector_camera;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->effector_camera.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->effector_camera.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return COMPUTE_EFFECTOR_CAMERA; };
    const char * getMD5(){ return "e28a9ea34e6e135a6309cbdf6fb0ad0d"; };

  };

  class compute_effector_camera {
    public:
    typedef compute_effector_cameraRequest Request;
    typedef compute_effector_cameraResponse Response;
  };

}
#endif
