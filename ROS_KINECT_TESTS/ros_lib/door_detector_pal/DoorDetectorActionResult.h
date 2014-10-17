#ifndef _ROS_door_detector_pal_DoorDetectorActionResult_h
#define _ROS_door_detector_pal_DoorDetectorActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "door_detector_pal/DoorDetectorResult.h"

namespace door_detector_pal
{

  class DoorDetectorActionResult : public ros::Msg
  {
    public:
      std_msgs::Header header;
      actionlib_msgs::GoalStatus status;
      door_detector_pal::DoorDetectorResult result;

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

    const char * getType(){ return "door_detector_pal/DoorDetectorActionResult"; };
    const char * getMD5(){ return "b2eaf8f60e7d93428e49616244b246b3"; };

  };

}
#endif