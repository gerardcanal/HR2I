#ifndef _ROS_pal_interaction_msgs_asrconfigOk_h
#define _ROS_pal_interaction_msgs_asrconfigOk_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pal_interaction_msgs
{

  class asrconfigOk : public ros::Msg
  {
    public:
      uint8_t conf_language_length;
      char* st_conf_language;
      char* * conf_language;
      uint8_t conf_enable_grammar_length;
      char* st_conf_enable_grammar;
      char* * conf_enable_grammar;
      uint8_t conf_disable_grammar_length;
      char* st_conf_disable_grammar;
      char* * conf_disable_grammar;
      uint8_t conf_acousticenv_length;
      char* st_conf_acousticenv;
      char* * conf_acousticenv;
      uint8_t conf_active_length;
      char* st_conf_active;
      char* * conf_active;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = conf_language_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < conf_language_length; i++){
      uint32_t length_conf_languagei = strlen( (const char*) this->conf_language[i]);
      memcpy(outbuffer + offset, &length_conf_languagei, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->conf_language[i], length_conf_languagei);
      offset += length_conf_languagei;
      }
      *(outbuffer + offset++) = conf_enable_grammar_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < conf_enable_grammar_length; i++){
      uint32_t length_conf_enable_grammari = strlen( (const char*) this->conf_enable_grammar[i]);
      memcpy(outbuffer + offset, &length_conf_enable_grammari, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->conf_enable_grammar[i], length_conf_enable_grammari);
      offset += length_conf_enable_grammari;
      }
      *(outbuffer + offset++) = conf_disable_grammar_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < conf_disable_grammar_length; i++){
      uint32_t length_conf_disable_grammari = strlen( (const char*) this->conf_disable_grammar[i]);
      memcpy(outbuffer + offset, &length_conf_disable_grammari, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->conf_disable_grammar[i], length_conf_disable_grammari);
      offset += length_conf_disable_grammari;
      }
      *(outbuffer + offset++) = conf_acousticenv_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < conf_acousticenv_length; i++){
      uint32_t length_conf_acousticenvi = strlen( (const char*) this->conf_acousticenv[i]);
      memcpy(outbuffer + offset, &length_conf_acousticenvi, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->conf_acousticenv[i], length_conf_acousticenvi);
      offset += length_conf_acousticenvi;
      }
      *(outbuffer + offset++) = conf_active_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < conf_active_length; i++){
      uint32_t length_conf_activei = strlen( (const char*) this->conf_active[i]);
      memcpy(outbuffer + offset, &length_conf_activei, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->conf_active[i], length_conf_activei);
      offset += length_conf_activei;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t conf_language_lengthT = *(inbuffer + offset++);
      if(conf_language_lengthT > conf_language_length)
        this->conf_language = (char**)realloc(this->conf_language, conf_language_lengthT * sizeof(char*));
      offset += 3;
      conf_language_length = conf_language_lengthT;
      for( uint8_t i = 0; i < conf_language_length; i++){
      uint32_t length_st_conf_language;
      memcpy(&length_st_conf_language, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_conf_language; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_conf_language-1]=0;
      this->st_conf_language = (char *)(inbuffer + offset-1);
      offset += length_st_conf_language;
        memcpy( &(this->conf_language[i]), &(this->st_conf_language), sizeof(char*));
      }
      uint8_t conf_enable_grammar_lengthT = *(inbuffer + offset++);
      if(conf_enable_grammar_lengthT > conf_enable_grammar_length)
        this->conf_enable_grammar = (char**)realloc(this->conf_enable_grammar, conf_enable_grammar_lengthT * sizeof(char*));
      offset += 3;
      conf_enable_grammar_length = conf_enable_grammar_lengthT;
      for( uint8_t i = 0; i < conf_enable_grammar_length; i++){
      uint32_t length_st_conf_enable_grammar;
      memcpy(&length_st_conf_enable_grammar, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_conf_enable_grammar; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_conf_enable_grammar-1]=0;
      this->st_conf_enable_grammar = (char *)(inbuffer + offset-1);
      offset += length_st_conf_enable_grammar;
        memcpy( &(this->conf_enable_grammar[i]), &(this->st_conf_enable_grammar), sizeof(char*));
      }
      uint8_t conf_disable_grammar_lengthT = *(inbuffer + offset++);
      if(conf_disable_grammar_lengthT > conf_disable_grammar_length)
        this->conf_disable_grammar = (char**)realloc(this->conf_disable_grammar, conf_disable_grammar_lengthT * sizeof(char*));
      offset += 3;
      conf_disable_grammar_length = conf_disable_grammar_lengthT;
      for( uint8_t i = 0; i < conf_disable_grammar_length; i++){
      uint32_t length_st_conf_disable_grammar;
      memcpy(&length_st_conf_disable_grammar, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_conf_disable_grammar; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_conf_disable_grammar-1]=0;
      this->st_conf_disable_grammar = (char *)(inbuffer + offset-1);
      offset += length_st_conf_disable_grammar;
        memcpy( &(this->conf_disable_grammar[i]), &(this->st_conf_disable_grammar), sizeof(char*));
      }
      uint8_t conf_acousticenv_lengthT = *(inbuffer + offset++);
      if(conf_acousticenv_lengthT > conf_acousticenv_length)
        this->conf_acousticenv = (char**)realloc(this->conf_acousticenv, conf_acousticenv_lengthT * sizeof(char*));
      offset += 3;
      conf_acousticenv_length = conf_acousticenv_lengthT;
      for( uint8_t i = 0; i < conf_acousticenv_length; i++){
      uint32_t length_st_conf_acousticenv;
      memcpy(&length_st_conf_acousticenv, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_conf_acousticenv; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_conf_acousticenv-1]=0;
      this->st_conf_acousticenv = (char *)(inbuffer + offset-1);
      offset += length_st_conf_acousticenv;
        memcpy( &(this->conf_acousticenv[i]), &(this->st_conf_acousticenv), sizeof(char*));
      }
      uint8_t conf_active_lengthT = *(inbuffer + offset++);
      if(conf_active_lengthT > conf_active_length)
        this->conf_active = (char**)realloc(this->conf_active, conf_active_lengthT * sizeof(char*));
      offset += 3;
      conf_active_length = conf_active_lengthT;
      for( uint8_t i = 0; i < conf_active_length; i++){
      uint32_t length_st_conf_active;
      memcpy(&length_st_conf_active, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_conf_active; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_conf_active-1]=0;
      this->st_conf_active = (char *)(inbuffer + offset-1);
      offset += length_st_conf_active;
        memcpy( &(this->conf_active[i]), &(this->st_conf_active), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return "pal_interaction_msgs/asrconfigOk"; };
    const char * getMD5(){ return "dddd3abd3a91b13b106138f740059454"; };

  };

}
#endif