#ifndef _ROS_humanoid_nav_msgs_ExecFootstepsFeedback_h
#define _ROS_humanoid_nav_msgs_ExecFootstepsFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "humanoid_nav_msgs/StepTarget.h"

namespace humanoid_nav_msgs
{

  class ExecFootstepsFeedback : public ros::Msg
  {
    public:
      uint8_t executed_footsteps_length;
      humanoid_nav_msgs::StepTarget st_executed_footsteps;
      humanoid_nav_msgs::StepTarget * executed_footsteps;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = executed_footsteps_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < executed_footsteps_length; i++){
      offset += this->executed_footsteps[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t executed_footsteps_lengthT = *(inbuffer + offset++);
      if(executed_footsteps_lengthT > executed_footsteps_length)
        this->executed_footsteps = (humanoid_nav_msgs::StepTarget*)realloc(this->executed_footsteps, executed_footsteps_lengthT * sizeof(humanoid_nav_msgs::StepTarget));
      offset += 3;
      executed_footsteps_length = executed_footsteps_lengthT;
      for( uint8_t i = 0; i < executed_footsteps_length; i++){
      offset += this->st_executed_footsteps.deserialize(inbuffer + offset);
        memcpy( &(this->executed_footsteps[i]), &(this->st_executed_footsteps), sizeof(humanoid_nav_msgs::StepTarget));
      }
     return offset;
    }

    const char * getType(){ return "humanoid_nav_msgs/ExecFootstepsFeedback"; };
    const char * getMD5(){ return "5dfde2cb244d6c76567d3c52c40a988c"; };

  };

}
#endif