#ifndef _ROS_nao_msgs_JointTrajectoryAction_h
#define _ROS_nao_msgs_JointTrajectoryAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "nao_msgs/JointTrajectoryActionGoal.h"
#include "nao_msgs/JointTrajectoryActionResult.h"
#include "nao_msgs/JointTrajectoryActionFeedback.h"

namespace nao_msgs
{

  class JointTrajectoryAction : public ros::Msg
  {
    public:
      nao_msgs::JointTrajectoryActionGoal action_goal;
      nao_msgs::JointTrajectoryActionResult action_result;
      nao_msgs::JointTrajectoryActionFeedback action_feedback;

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

    const char * getType(){ return "nao_msgs/JointTrajectoryAction"; };
    const char * getMD5(){ return "a88e50dd366fa304954dd433b5706bd8"; };

  };

}
#endif