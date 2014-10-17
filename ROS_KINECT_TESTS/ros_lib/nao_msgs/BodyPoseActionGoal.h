#ifndef _ROS_nao_msgs_BodyPoseActionGoal_h
#define _ROS_nao_msgs_BodyPoseActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "nao_msgs/BodyPoseGoal.h"

namespace nao_msgs
{

  class BodyPoseActionGoal : public ros::Msg
  {
    public:
      std_msgs::Header header;
      actionlib_msgs::GoalID goal_id;
      nao_msgs::BodyPoseGoal goal;

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

    const char * getType(){ return "nao_msgs/BodyPoseActionGoal"; };
    const char * getMD5(){ return "0c4ae1487ff4d033a7fa048a0b31509c"; };

  };

}
#endif