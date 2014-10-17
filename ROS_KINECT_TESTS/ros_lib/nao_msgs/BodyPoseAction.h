#ifndef _ROS_nao_msgs_BodyPoseAction_h
#define _ROS_nao_msgs_BodyPoseAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "nao_msgs/BodyPoseActionGoal.h"
#include "nao_msgs/BodyPoseActionResult.h"
#include "nao_msgs/BodyPoseActionFeedback.h"

namespace nao_msgs
{

  class BodyPoseAction : public ros::Msg
  {
    public:
      nao_msgs::BodyPoseActionGoal action_goal;
      nao_msgs::BodyPoseActionResult action_result;
      nao_msgs::BodyPoseActionFeedback action_feedback;

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

    const char * getType(){ return "nao_msgs/BodyPoseAction"; };
    const char * getMD5(){ return "bc58fd1218bb64624b76dca7ad8a5a81"; };

  };

}
#endif