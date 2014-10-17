#ifndef _ROS_nao_msgs_JointAnglesWithSpeedAction_h
#define _ROS_nao_msgs_JointAnglesWithSpeedAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "nao_msgs/JointAnglesWithSpeedActionGoal.h"
#include "nao_msgs/JointAnglesWithSpeedActionResult.h"
#include "nao_msgs/JointAnglesWithSpeedActionFeedback.h"

namespace nao_msgs
{

  class JointAnglesWithSpeedAction : public ros::Msg
  {
    public:
      nao_msgs::JointAnglesWithSpeedActionGoal action_goal;
      nao_msgs::JointAnglesWithSpeedActionResult action_result;
      nao_msgs::JointAnglesWithSpeedActionFeedback action_feedback;

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

    const char * getType(){ return "nao_msgs/JointAnglesWithSpeedAction"; };
    const char * getMD5(){ return "efd2f7ac88847414fd26aacf32f993a5"; };

  };

}
#endif