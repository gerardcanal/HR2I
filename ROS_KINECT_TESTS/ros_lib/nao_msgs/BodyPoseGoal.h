#ifndef _ROS_nao_msgs_BodyPoseGoal_h
#define _ROS_nao_msgs_BodyPoseGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace nao_msgs
{

  class BodyPoseGoal : public ros::Msg
  {
    public:
      char * pose_name;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_pose_name = strlen( (const char*) this->pose_name);
      memcpy(outbuffer + offset, &length_pose_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->pose_name, length_pose_name);
      offset += length_pose_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_pose_name;
      memcpy(&length_pose_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_pose_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_pose_name-1]=0;
      this->pose_name = (char *)(inbuffer + offset-1);
      offset += length_pose_name;
     return offset;
    }

    const char * getType(){ return "nao_msgs/BodyPoseGoal"; };
    const char * getMD5(){ return "e6184073e8e665fb2bf0be194fc36541"; };

  };

}
#endif