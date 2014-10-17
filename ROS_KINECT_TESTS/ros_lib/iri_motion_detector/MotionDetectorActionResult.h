#ifndef _ROS_iri_motion_detector_MotionDetectorActionResult_h
#define _ROS_iri_motion_detector_MotionDetectorActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"

namespace iri_motion_detector
{

  class MotionDetectorActionResult : public ros::Msg
  {
    public:
      geometry_msgs::PoseStamped pose;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "iri_motion_detector/MotionDetectorActionResult"; };
    const char * getMD5(){ return "3f8930d968a3e84d471dff917bb1cdae"; };

  };

}
#endif