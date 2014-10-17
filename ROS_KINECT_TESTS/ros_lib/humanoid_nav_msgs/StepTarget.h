#ifndef _ROS_humanoid_nav_msgs_StepTarget_h
#define _ROS_humanoid_nav_msgs_StepTarget_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose2D.h"

namespace humanoid_nav_msgs
{

  class StepTarget : public ros::Msg
  {
    public:
      geometry_msgs::Pose2D pose;
      uint8_t leg;
      enum { right = 0                };
      enum { left = 1                 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pose.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->leg >> (8 * 0)) & 0xFF;
      offset += sizeof(this->leg);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pose.deserialize(inbuffer + offset);
      this->leg =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->leg);
     return offset;
    }

    const char * getType(){ return "humanoid_nav_msgs/StepTarget"; };
    const char * getMD5(){ return "8ccf34ddb67039fbda0d9b2515ebb1ea"; };

  };

}
#endif