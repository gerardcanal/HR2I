#ifndef _ROS_face_recognition_FaceRecognitionResult_h
#define _ROS_face_recognition_FaceRecognitionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace face_recognition
{

  class FaceRecognitionResult : public ros::Msg
  {
    public:
      uint8_t order_id;
      uint8_t names_length;
      char* st_names;
      char* * names;
      uint8_t confidence_length;
      float st_confidence;
      float * confidence;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->order_id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->order_id);
      *(outbuffer + offset++) = names_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < names_length; i++){
      uint32_t length_namesi = strlen( (const char*) this->names[i]);
      memcpy(outbuffer + offset, &length_namesi, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->names[i], length_namesi);
      offset += length_namesi;
      }
      *(outbuffer + offset++) = confidence_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < confidence_length; i++){
      union {
        float real;
        uint32_t base;
      } u_confidencei;
      u_confidencei.real = this->confidence[i];
      *(outbuffer + offset + 0) = (u_confidencei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_confidencei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_confidencei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_confidencei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->confidence[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->order_id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->order_id);
      uint8_t names_lengthT = *(inbuffer + offset++);
      if(names_lengthT > names_length)
        this->names = (char**)realloc(this->names, names_lengthT * sizeof(char*));
      offset += 3;
      names_length = names_lengthT;
      for( uint8_t i = 0; i < names_length; i++){
      uint32_t length_st_names;
      memcpy(&length_st_names, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_names-1]=0;
      this->st_names = (char *)(inbuffer + offset-1);
      offset += length_st_names;
        memcpy( &(this->names[i]), &(this->st_names), sizeof(char*));
      }
      uint8_t confidence_lengthT = *(inbuffer + offset++);
      if(confidence_lengthT > confidence_length)
        this->confidence = (float*)realloc(this->confidence, confidence_lengthT * sizeof(float));
      offset += 3;
      confidence_length = confidence_lengthT;
      for( uint8_t i = 0; i < confidence_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_confidence;
      u_st_confidence.base = 0;
      u_st_confidence.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_confidence.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_confidence.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_confidence.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_confidence = u_st_confidence.real;
      offset += sizeof(this->st_confidence);
        memcpy( &(this->confidence[i]), &(this->st_confidence), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "face_recognition/FaceRecognitionResult"; };
    const char * getMD5(){ return "1354b7478703dcb5d9e033b8bdfafb3b"; };

  };

}
#endif