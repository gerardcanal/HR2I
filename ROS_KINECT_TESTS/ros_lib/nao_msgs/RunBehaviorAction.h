#ifndef _ROS_nao_msgs_RunBehaviorAction_h
#define _ROS_nao_msgs_RunBehaviorAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "nao_msgs/RunBehaviorActionGoal.h"
#include "nao_msgs/RunBehaviorActionResult.h"
#include "nao_msgs/RunBehaviorActionFeedback.h"

namespace nao_msgs
{

  class RunBehaviorAction : public ros::Msg
  {
    public:
      nao_msgs::RunBehaviorActionGoal action_goal;
      nao_msgs::RunBehaviorActionResult action_result;
      nao_msgs::RunBehaviorActionFeedback action_feedback;

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

    const char * getType(){ return "nao_msgs/RunBehaviorAction"; };
    const char * getMD5(){ return "a6a26afb8ff0902c5587c8bfbdc46892"; };

  };

}
#endif