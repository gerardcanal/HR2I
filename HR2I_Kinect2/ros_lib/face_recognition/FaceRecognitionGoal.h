#ifndef _ROS_face_recognition_FaceRecognitionGoal_h
#define _ROS_face_recognition_FaceRecognitionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace face_recognition
{

  class FaceRecognitionGoal : public ros::Msg
  {
    public:
      uint8_t order_id;
      char * order_argument;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->order_id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->order_id);
      uint32_t length_order_argument = strlen( (const char*) this->order_argument);
      memcpy(outbuffer + offset, &length_order_argument, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->order_argument, length_order_argument);
      offset += length_order_argument;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->order_id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->order_id);
      uint32_t length_order_argument;
      memcpy(&length_order_argument, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_order_argument; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_order_argument-1]=0;
      this->order_argument = (char *)(inbuffer + offset-1);
      offset += length_order_argument;
     return offset;
    }

    const char * getType(){ return "face_recognition/FaceRecognitionGoal"; };
    const char * getMD5(){ return "12fa0d0af9b141eceafa42011d31f9c2"; };

  };

}
#endif