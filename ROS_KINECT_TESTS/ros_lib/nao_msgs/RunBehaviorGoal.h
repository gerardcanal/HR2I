#ifndef _ROS_nao_msgs_RunBehaviorGoal_h
#define _ROS_nao_msgs_RunBehaviorGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace nao_msgs
{

  class RunBehaviorGoal : public ros::Msg
  {
    public:
      char * behavior;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_behavior = strlen( (const char*) this->behavior);
      memcpy(outbuffer + offset, &length_behavior, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->behavior, length_behavior);
      offset += length_behavior;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_behavior;
      memcpy(&length_behavior, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_behavior; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_behavior-1]=0;
      this->behavior = (char *)(inbuffer + offset-1);
      offset += length_behavior;
     return offset;
    }

    const char * getType(){ return "nao_msgs/RunBehaviorGoal"; };
    const char * getMD5(){ return "03729983c4b9be7a4f2b56846a7ccbdc"; };

  };

}
#endif