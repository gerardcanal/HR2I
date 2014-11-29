#ifndef _ROS_pr_msgs_AppletState_h
#define _ROS_pr_msgs_AppletState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr_msgs/Action.h"

namespace pr_msgs
{

  class AppletState : public ros::Msg
  {
    public:
      char * cmd_topic_name;
      uint8_t actions_length;
      pr_msgs::Action st_actions;
      pr_msgs::Action * actions;
      uint8_t state;
      char * info;
      enum { state_idle = 0 };
      enum { state_busy = 1 };
      enum { state_error = 2 };
      enum { state_dying = 3 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_cmd_topic_name = strlen( (const char*) this->cmd_topic_name);
      memcpy(outbuffer + offset, &length_cmd_topic_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->cmd_topic_name, length_cmd_topic_name);
      offset += length_cmd_topic_name;
      *(outbuffer + offset++) = actions_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < actions_length; i++){
      offset += this->actions[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      uint32_t length_info = strlen( (const char*) this->info);
      memcpy(outbuffer + offset, &length_info, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->info, length_info);
      offset += length_info;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_cmd_topic_name;
      memcpy(&length_cmd_topic_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_cmd_topic_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_cmd_topic_name-1]=0;
      this->cmd_topic_name = (char *)(inbuffer + offset-1);
      offset += length_cmd_topic_name;
      uint8_t actions_lengthT = *(inbuffer + offset++);
      if(actions_lengthT > actions_length)
        this->actions = (pr_msgs::Action*)realloc(this->actions, actions_lengthT * sizeof(pr_msgs::Action));
      offset += 3;
      actions_length = actions_lengthT;
      for( uint8_t i = 0; i < actions_length; i++){
      offset += this->st_actions.deserialize(inbuffer + offset);
        memcpy( &(this->actions[i]), &(this->st_actions), sizeof(pr_msgs::Action));
      }
      this->state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->state);
      uint32_t length_info;
      memcpy(&length_info, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_info; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_info-1]=0;
      this->info = (char *)(inbuffer + offset-1);
      offset += length_info;
     return offset;
    }

    const char * getType(){ return "pr_msgs/AppletState"; };
    const char * getMD5(){ return "7c39cb5217576cbaaa8bc739f08abe33"; };

  };

}
#endif