#ifndef _ROS_pr_msgs_oldAppletCommand_h
#define _ROS_pr_msgs_oldAppletCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

  class oldAppletCommand : public ros::Msg
  {
    public:
      char * node_name;
      char * action_name;
      char * target_name;
      int32_t command_id;
      int32_t prep_timelimit;
      int32_t execution_timelimit;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_node_name = strlen( (const char*) this->node_name);
      memcpy(outbuffer + offset, &length_node_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->node_name, length_node_name);
      offset += length_node_name;
      uint32_t length_action_name = strlen( (const char*) this->action_name);
      memcpy(outbuffer + offset, &length_action_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->action_name, length_action_name);
      offset += length_action_name;
      uint32_t length_target_name = strlen( (const char*) this->target_name);
      memcpy(outbuffer + offset, &length_target_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->target_name, length_target_name);
      offset += length_target_name;
      union {
        int32_t real;
        uint32_t base;
      } u_command_id;
      u_command_id.real = this->command_id;
      *(outbuffer + offset + 0) = (u_command_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_command_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_command_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_command_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->command_id);
      union {
        int32_t real;
        uint32_t base;
      } u_prep_timelimit;
      u_prep_timelimit.real = this->prep_timelimit;
      *(outbuffer + offset + 0) = (u_prep_timelimit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_prep_timelimit.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_prep_timelimit.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_prep_timelimit.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->prep_timelimit);
      union {
        int32_t real;
        uint32_t base;
      } u_execution_timelimit;
      u_execution_timelimit.real = this->execution_timelimit;
      *(outbuffer + offset + 0) = (u_execution_timelimit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_execution_timelimit.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_execution_timelimit.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_execution_timelimit.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->execution_timelimit);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_node_name;
      memcpy(&length_node_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_node_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_node_name-1]=0;
      this->node_name = (char *)(inbuffer + offset-1);
      offset += length_node_name;
      uint32_t length_action_name;
      memcpy(&length_action_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_action_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_action_name-1]=0;
      this->action_name = (char *)(inbuffer + offset-1);
      offset += length_action_name;
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
      } u_command_id;
      u_command_id.base = 0;
      u_command_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_command_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_command_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_command_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->command_id = u_command_id.real;
      offset += sizeof(this->command_id);
      union {
        int32_t real;
        uint32_t base;
      } u_prep_timelimit;
      u_prep_timelimit.base = 0;
      u_prep_timelimit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_prep_timelimit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_prep_timelimit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_prep_timelimit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->prep_timelimit = u_prep_timelimit.real;
      offset += sizeof(this->prep_timelimit);
      union {
        int32_t real;
        uint32_t base;
      } u_execution_timelimit;
      u_execution_timelimit.base = 0;
      u_execution_timelimit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_execution_timelimit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_execution_timelimit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_execution_timelimit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->execution_timelimit = u_execution_timelimit.real;
      offset += sizeof(this->execution_timelimit);
     return offset;
    }

    const char * getType(){ return "pr_msgs/oldAppletCommand"; };
    const char * getMD5(){ return "3d5ec065e60aaea41600679f9f3cdd68"; };

  };

}
#endif