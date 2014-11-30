#ifndef _ROS_naoqi_msgs_JointTrajectoryGoal_h
#define _ROS_naoqi_msgs_JointTrajectoryGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "trajectory_msgs/JointTrajectory.h"

namespace naoqi_msgs
{

  class JointTrajectoryGoal : public ros::Msg
  {
    public:
      trajectory_msgs::JointTrajectory trajectory;
      uint8_t relative;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->trajectory.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->relative >> (8 * 0)) & 0xFF;
      offset += sizeof(this->relative);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->trajectory.deserialize(inbuffer + offset);
      this->relative =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->relative);
     return offset;
    }

    const char * getType(){ return "naoqi_msgs/JointTrajectoryGoal"; };
    const char * getMD5(){ return "7ecdd56459ac4b8e2c210a74dbb66523"; };

  };

}
#endif