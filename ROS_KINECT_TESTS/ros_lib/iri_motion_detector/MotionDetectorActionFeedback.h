#ifndef _ROS_iri_motion_detector_MotionDetectorActionFeedback_h
#define _ROS_iri_motion_detector_MotionDetectorActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace iri_motion_detector
{

  class MotionDetectorActionFeedback : public ros::Msg
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

    const char * getType(){ return "iri_motion_detector/MotionDetectorActionFeedback"; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif