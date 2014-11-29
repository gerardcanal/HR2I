#ifndef _ROS_door_detector_pal_DoorDetectorAction_h
#define _ROS_door_detector_pal_DoorDetectorAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "door_detector_pal/DoorDetectorActionGoal.h"
#include "door_detector_pal/DoorDetectorActionResult.h"
#include "door_detector_pal/DoorDetectorActionFeedback.h"

namespace door_detector_pal
{

  class DoorDetectorAction : public ros::Msg
  {
    public:
      door_detector_pal::DoorDetectorActionGoal action_goal;
      door_detector_pal::DoorDetectorActionResult action_result;
      door_detector_pal::DoorDetectorActionFeedback action_feedback;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "door_detector_pal/DoorDetectorAction"; };
    const char * getMD5(){ return "1d11563d2074254a916bac01d478f223"; };

  };

}
#endif