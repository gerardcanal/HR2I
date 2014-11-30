#ifndef _ROS_SERVICE_compute_effector_camera_quick_h
#define _ROS_SERVICE_compute_effector_camera_quick_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "visp_hand2eye_calibration/TransformArray.h"
#include "geometry_msgs/Transform.h"

namespace visp_hand2eye_calibration
{

static const char COMPUTE_EFFECTOR_CAMERA_QUICK[] = "visp_hand2eye_calibration/compute_effector_camera_quick";

  class compute_effector_camera_quickRequest : public ros::Msg
  {
    public:
      visp_hand2eye_calibration::TransformArray camera_object;
      visp_hand2eye_calibration::TransformArray world_effector;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->camera_object.serialize(outbuffer + offset);
      offset += this->world_effector.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->camera_object.deserialize(inbuffer + offset);
      offset += this->world_effector.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return COMPUTE_EFFECTOR_CAMERA_QUICK; };
    const char * getMD5(){ return "4ce42b1236ce32724031d598c2a332bf"; };

  };

  class compute_effector_camera_quickResponse : public ros::Msg
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

    const char * getType(){ return COMPUTE_EFFECTOR_CAMERA_QUICK; };
    const char * getMD5(){ return "e28a9ea34e6e135a6309cbdf6fb0ad0d"; };

  };

  class compute_effector_camera_quick {
    public:
    typedef compute_effector_camera_quickRequest Request;
    typedef compute_effector_camera_quickResponse Response;
  };

}
#endif
