#ifndef _ROS_pr_msgs_oldAppletState_h
#define _ROS_pr_msgs_oldAppletState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr_msgs/Action.h"

namespace pr_msgs
{

  class oldAppletState : public ros::Msg
  {
    public:
      uint8_t actions_length;
      pr_msgs::Action st_actions;
      pr_msgs::Action * actions;
      uint8_t state;
      int32_t last_command_id;
      char * node_name;
      char * info;
      enum { state_idle = 0 };
      enum { state_busy = 1 };
      enum { state_error = 2 };
      enum { state_dying = 3 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = actions_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < actions_length; i++){
      offset += this->actions[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      union {
        int32_t real;
        uint32_t base;
      } u_last_command_id;
      u_last_command_id.real = this->last_command_id;
      *(outbuffer + offset + 0) = (u_last_command_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_last_command_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_last_command_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_last_command_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->last_command_id);
      uint32_t length_node_name = strlen( (const char*) this->node_name);
      memcpy(outbuffer + offset, &length_node_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->node_name, length_node_name);
      offset += length_node_name;
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
      union {
        int32_t real;
        uint32_t base;
      } u_last_command_id;
      u_last_command_id.base = 0;
      u_last_command_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_last_command_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_last_command_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_last_command_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->last_command_id = u_last_command_id.real;
      offset += sizeof(this->last_command_id);
      uint32_t length_node_name;
      memcpy(&length_node_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_node_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_node_name-1]=0;
      this->node_name = (char *)(inbuffer + offset-1);
      offset += length_node_name;
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

    const char * getType(){ return "pr_msgs/oldAppletState"; };
    const char * getMD5(){ return "1fe9abd2f3ee03d21319d51ece2576db"; };

  };

}
#endif