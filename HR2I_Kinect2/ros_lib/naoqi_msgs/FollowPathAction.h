#ifndef _ROS_naoqi_msgs_FollowPathAction_h
#define _ROS_naoqi_msgs_FollowPathAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "naoqi_msgs/FollowPathActionGoal.h"
#include "naoqi_msgs/FollowPathActionResult.h"
#include "naoqi_msgs/FollowPathActionFeedback.h"

namespace naoqi_msgs
{

  class FollowPathAction : public ros::Msg
  {
    public:
      naoqi_msgs::FollowPathActionGoal action_goal;
      naoqi_msgs::FollowPathActionResult action_result;
      naoqi_msgs::FollowPathActionFeedback action_feedback;

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

    const char * getType(){ return "naoqi_msgs/FollowPathAction"; };
    const char * getMD5(){ return "98958d560f45913f6e3143ad99e2fcf0"; };

  };

}
#endif