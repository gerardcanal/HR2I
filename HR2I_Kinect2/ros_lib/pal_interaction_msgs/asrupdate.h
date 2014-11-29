#ifndef _ROS_pal_interaction_msgs_asrupdate_h
#define _ROS_pal_interaction_msgs_asrupdate_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pal_interaction_msgs
{

  class asrupdate : public ros::Msg
  {
    public:
      char * language;
      char * enable_grammar;
      char * disable_grammar;
      char * acousticenv;
      bool active;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_language = strlen( (const char*) this->language);
      memcpy(outbuffer + offset, &length_language, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->language, length_language);
      offset += length_language;
      uint32_t length_enable_grammar = strlen( (const char*) this->enable_grammar);
      memcpy(outbuffer + offset, &length_enable_grammar, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->enable_grammar, length_enable_grammar);
      offset += length_enable_grammar;
      uint32_t length_disable_grammar = strlen( (const char*) this->disable_grammar);
      memcpy(outbuffer + offset, &length_disable_grammar, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->disable_grammar, length_disable_grammar);
      offset += length_disable_grammar;
      uint32_t length_acousticenv = strlen( (const char*) this->acousticenv);
      memcpy(outbuffer + offset, &length_acousticenv, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->acousticenv, length_acousticenv);
      offset += length_acousticenv;
      union {
        bool real;
        uint8_t base;
      } u_active;
      u_active.real = this->active;
      *(outbuffer + offset + 0) = (u_active.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->active);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_language;
      memcpy(&length_language, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_language; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_language-1]=0;
      this->language = (char *)(inbuffer + offset-1);
      offset += length_language;
      uint32_t length_enable_grammar;
      memcpy(&length_enable_grammar, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_enable_grammar; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_enable_grammar-1]=0;
      this->enable_grammar = (char *)(inbuffer + offset-1);
      offset += length_enable_grammar;
      uint32_t length_disable_grammar;
      memcpy(&length_disable_grammar, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_disable_grammar; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_disable_grammar-1]=0;
      this->disable_grammar = (char *)(inbuffer + offset-1);
      offset += length_disable_grammar;
      uint32_t length_acousticenv;
      memcpy(&length_acousticenv, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_acousticenv; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_acousticenv-1]=0;
      this->acousticenv = (char *)(inbuffer + offset-1);
      offset += length_acousticenv;
      union {
        bool real;
        uint8_t base;
      } u_active;
      u_active.base = 0;
      u_active.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->active = u_active.real;
      offset += sizeof(this->active);
     return offset;
    }

    const char * getType(){ return "pal_interaction_msgs/asrupdate"; };
    const char * getMD5(){ return "1f28e753977d8fe335d6b968a649f1ed"; };

  };

}
#endif