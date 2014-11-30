#ifndef _ROS_naoqi_msgs_SetSpeechVocabularyGoal_h
#define _ROS_naoqi_msgs_SetSpeechVocabularyGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace naoqi_msgs
{

  class SetSpeechVocabularyGoal : public ros::Msg
  {
    public:
      uint8_t words_length;
      char* st_words;
      char* * words;

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
     return offset;
    }

    const char * getType(){ return "naoqi_msgs/SetSpeechVocabularyGoal"; };
    const char * getMD5(){ return "2bd0e7dd008cf8f52a5113ba090403b7"; };

  };

}
#endif