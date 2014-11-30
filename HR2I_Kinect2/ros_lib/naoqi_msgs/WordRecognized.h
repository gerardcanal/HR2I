#ifndef _ROS_naoqi_msgs_WordRecognized_h
#define _ROS_naoqi_msgs_WordRecognized_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace naoqi_msgs
{

  class WordRecognized : public ros::Msg
  {
    public:
      uint8_t words_length;
      char* st_words;
      char* * words;
      uint8_t confidence_values_length;
      float st_confidence_values;
      float * confidence_values;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = words_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < words_length; i++){
      uint32_t length_wordsi = strlen(this->words[i]);
      memcpy(outbuffer + offset, &length_wordsi, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->words[i], length_wordsi);
      offset += length_wordsi;
      }
      *(outbuffer + offset++) = confidence_values_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < confidence_values_length; i++){
      union {
        float real;
        uint32_t base;
      } u_confidence_valuesi;
      u_confidence_valuesi.real = this->confidence_values[i];
      *(outbuffer + offset + 0) = (u_confidence_valuesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_confidence_valuesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_confidence_valuesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_confidence_valuesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->confidence_values[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t words_lengthT = *(inbuffer + offset++);
      if(words_lengthT > words_length)
        this->words = (char**)realloc(this->words, words_lengthT * sizeof(char*));
      offset += 3;
      words_length = words_lengthT;
      for( uint8_t i = 0; i < words_length; i++){
      uint32_t length_st_words;
      memcpy(&length_st_words, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_words; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_words-1]=0;
      this->st_words = (char *)(inbuffer + offset-1);
      offset += length_st_words;
        memcpy( &(this->words[i]), &(this->st_words), sizeof(char*));
      }
      uint8_t confidence_values_lengthT = *(inbuffer + offset++);
      if(confidence_values_lengthT > confidence_values_length)
        this->confidence_values = (float*)realloc(this->confidence_values, confidence_values_lengthT * sizeof(float));
      offset += 3;
      confidence_values_length = confidence_values_lengthT;
      for( uint8_t i = 0; i < confidence_values_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_confidence_values;
      u_st_confidence_values.base = 0;
      u_st_confidence_values.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_confidence_values.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_confidence_values.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_confidence_values.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_confidence_values = u_st_confidence_values.real;
      offset += sizeof(this->st_confidence_values);
        memcpy( &(this->confidence_values[i]), &(this->st_confidence_values), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "naoqi_msgs/WordRecognized"; };
    const char * getMD5(){ return "29134437cd61021f75f35f21b72b7eab"; };

  };

}
#endif