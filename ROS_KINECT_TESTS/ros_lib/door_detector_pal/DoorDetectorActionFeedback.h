#ifndef _ROS_door_detector_pal_DoorDetectorActionFeedback_h
#define _ROS_door_detector_pal_DoorDetectorActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "door_detector_pal/DoorDetectorFeedback.h"

namespace door_detector_pal
{

  class DoorDetectorActionFeedback : public ros::Msg
  {
    public:
      std_msgs::Header header;
      actionlib_msgs::GoalStatus status;
      door_detector_pal::DoorDetectorFeedback feedback;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "door_detector_pal/DoorDetectorActionFeedback"; };
    const char * getMD5(){ return "2dbfd4fc9af5dafafe0c2843a1e4ce24"; };

  };

}
#endif