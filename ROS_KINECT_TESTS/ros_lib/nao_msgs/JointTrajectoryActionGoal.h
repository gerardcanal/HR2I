#ifndef _ROS_nao_msgs_JointTrajectoryActionGoal_h
#define _ROS_nao_msgs_JointTrajectoryActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "nao_msgs/JointTrajectoryGoal.h"

namespace nao_msgs
{

  class JointTrajectoryActionGoal : public ros::Msg
  {
    public:
      std_msgs::Header header;
      actionlib_msgs::GoalID goal_id;
      nao_msgs::JointTrajectoryGoal goal;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->goal_id.serialize(outbuffer + offset);
      offset += this->goal.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->goal_id.deserialize(inbuffer + offset);
      offset += this->goal.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "nao_msgs/JointTrajectoryActionGoal"; };
    const char * getMD5(){ return "01c0441c9496ef36b8677931c016db7f"; };

  };

}
#endif