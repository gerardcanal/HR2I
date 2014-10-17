#ifndef _ROS_pr_msgs_SignalAction_h
#define _ROS_pr_msgs_SignalAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr_msgs/SignalActionGoal.h"
#include "pr_msgs/SignalActionResult.h"
#include "pr_msgs/SignalActionFeedback.h"

namespace pr_msgs
{

  class SignalAction : public ros::Msg
  {
    public:
      pr_msgs::SignalActionGoal action_goal;
      pr_msgs::SignalActionResult action_result;
      pr_msgs::SignalActionFeedback action_feedback;

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

    const char * getType(){ return "pr_msgs/SignalAction"; };
    const char * getMD5(){ return "d5a016b49f278075666fbc901debbd08"; };

  };

}
#endif