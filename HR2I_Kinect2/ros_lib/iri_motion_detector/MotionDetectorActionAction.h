#ifndef _ROS_iri_motion_detector_MotionDetectorActionAction_h
#define _ROS_iri_motion_detector_MotionDetectorActionAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "iri_motion_detector/MotionDetectorActionActionGoal.h"
#include "iri_motion_detector/MotionDetectorActionActionResult.h"
#include "iri_motion_detector/MotionDetectorActionActionFeedback.h"

namespace iri_motion_detector
{

  class MotionDetectorActionAction : public ros::Msg
  {
    public:
      iri_motion_detector::MotionDetectorActionActionGoal action_goal;
      iri_motion_detector::MotionDetectorActionActionResult action_result;
      iri_motion_detector::MotionDetectorActionActionFeedback action_feedback;

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

    const char * getType(){ return "iri_motion_detector/MotionDetectorActionAction"; };
    const char * getMD5(){ return "7eb336d3ca9736b88c7a244db729f927"; };

  };

}
#endif