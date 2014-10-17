#ifndef _ROS_pr_msgs_Action_h
#define _ROS_pr_msgs_Action_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

  class Action : public ros::Msg
  {
    public:
      char * name;
      char * target_name;
      int32_t prep_duration;
      int32_t execution_duration;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen( (const char*) this->name);
      memcpy(outbuffer + offset, &length_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_target_name = strlen( (const char*) this->target_name);
      memcpy(outbuffer + offset, &length_target_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->target_name, length_target_name);
      offset += length_target_name;
      union {
        int32_t real;
        uint32_t base;
      } u_prep_duration;
      u_prep_duration.real = this->prep_duration;
      *(outbuffer + offset + 0) = (u_prep_duration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_prep_duration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_prep_duration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_prep_duration.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->prep_duration);
      union {
        int32_t real;
        uint32_t base;
      } u_execution_duration;
      u_execution_duration.real = this->execution_duration;
      *(outbuffer + offset + 0) = (u_execution_duration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_execution_duration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_execution_duration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_execution_duration.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->execution_duration);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      memcpy(&length_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_target_name;
      memcpy(&length_target_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_target_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_target_name-1]=0;
      this->target_name = (char *)(inbuffer + offset-1);
      offset += length_target_name;
      union {
        int32_t real;
        uint32_t base;
      } u_prep_duration;
      u_prep_duration.base = 0;
      u_prep_duration.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_prep_duration.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_prep_duration.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_prep_duration.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->prep_duration = u_prep_duration.real;
      offset += sizeof(this->prep_duration);
      union {
        int32_t real;
        uint32_t base;
      } u_execution_duration;
      u_execution_duration.base = 0;
      u_execution_duration.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_execution_duration.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_execution_duration.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_execution_duration.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->execution_duration = u_execution_duration.real;
      offset += sizeof(this->execution_duration);
     return offset;
    }

    const char * getType(){ return "pr_msgs/Action"; };
    const char * getMD5(){ return "6ddf77916fbb7875e9cbd82630d8ea35"; };

  };

}
#endif