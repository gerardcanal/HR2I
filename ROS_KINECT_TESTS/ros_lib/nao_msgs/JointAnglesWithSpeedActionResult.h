#ifndef _ROS_nao_msgs_JointAnglesWithSpeedActionResult_h
#define _ROS_nao_msgs_JointAnglesWithSpeedActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "nao_msgs/JointAnglesWithSpeedResult.h"

namespace nao_msgs
{

  class JointAnglesWithSpeedActionResult : public ros::Msg
  {
    public:
      std_msgs::Header header;
      actionlib_msgs::GoalStatus status;
      nao_msgs::JointAnglesWithSpeedResult result;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "nao_msgs/JointAnglesWithSpeedActionResult"; };
    const char * getMD5(){ return "8863b007f420d5f94fcdaa0f865d1767"; };

  };

}
#endif