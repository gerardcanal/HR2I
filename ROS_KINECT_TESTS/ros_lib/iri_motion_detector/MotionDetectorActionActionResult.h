#ifndef _ROS_iri_motion_detector_MotionDetectorActionActionResult_h
#define _ROS_iri_motion_detector_MotionDetectorActionActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "iri_motion_detector/MotionDetectorActionResult.h"

namespace iri_motion_detector
{

  class MotionDetectorActionActionResult : public ros::Msg
  {
    public:
      std_msgs::Header header;
      actionlib_msgs::GoalStatus status;
      iri_motion_detector::MotionDetectorActionResult result;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "iri_motion_detector/MotionDetectorActionActionResult"; };
    const char * getMD5(){ return "6d0edd723ba2f2ae6bcfa76c98c311f7"; };

  };

}
#endif