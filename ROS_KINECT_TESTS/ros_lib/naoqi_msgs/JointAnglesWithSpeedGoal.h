#ifndef _ROS_naoqi_msgs_JointAnglesWithSpeedGoal_h
#define _ROS_naoqi_msgs_JointAnglesWithSpeedGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "naoqi_msgs/JointAnglesWithSpeed.h"

namespace naoqi_msgs
{

  class JointAnglesWithSpeedGoal : public ros::Msg
  {
    public:
      naoqi_msgs::JointAnglesWithSpeed joint_angles;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->joint_angles.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->joint_angles.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "naoqi_msgs/JointAnglesWithSpeedGoal"; };
    const char * getMD5(){ return "d19a898a40aae87b37b0f91c9e90f46c"; };

  };

}
#endif