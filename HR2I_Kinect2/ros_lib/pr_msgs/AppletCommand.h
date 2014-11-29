#ifndef _ROS_SERVICE_AppletCommand_h
#define _ROS_SERVICE_AppletCommand_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

static const char APPLETCOMMAND[] = "pr_msgs/AppletCommand";

  class AppletCommandRequest : public ros::Msg
  {
    public:
      char * action_name;
      char * target_name;
      int32_t prep_timelimit;
      int32_t execution_timelimit;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
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

    const char * getType(){ return APPLETCOMMAND; };
    const char * getMD5(){ return "e67ef5adf1683522d90687d5becf7233"; };

  };

  class AppletCommandResponse : public ros::Msg
  {
    public:
      uint8_t result;
      char * info;
      enum { DONE = 0 };
      enum { UNKNOWN_COMMAND = 1 };
      enum { UNAVAILABLE = 2 };
      enum { TIMEOUT = 3 };
      enum { ERROR = 4 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->result >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result);
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
      this->result =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->result);
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

    const char * getType(){ return APPLETCOMMAND; };
    const char * getMD5(){ return "e26da08e1081fe0c6d99808a122b6762"; };

  };

  class AppletCommand {
    public:
    typedef AppletCommandRequest Request;
    typedef AppletCommandResponse Response;
  };

}
#endif
