#ifndef _ROS_pr_msgs_QR_h
#define _ROS_pr_msgs_QR_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

  class QR : public ros::Msg
  {
    public:
      uint8_t qr_strings_length;
      char* st_qr_strings;
      char* * qr_strings;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = qr_strings_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < qr_strings_length; i++){
      uint32_t length_qr_stringsi = strlen( (const char*) this->qr_strings[i]);
      memcpy(outbuffer + offset, &length_qr_stringsi, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->qr_strings[i], length_qr_stringsi);
      offset += length_qr_stringsi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t qr_strings_lengthT = *(inbuffer + offset++);
      if(qr_strings_lengthT > qr_strings_length)
        this->qr_strings = (char**)realloc(this->qr_strings, qr_strings_lengthT * sizeof(char*));
      offset += 3;
      qr_strings_length = qr_strings_lengthT;
      for( uint8_t i = 0; i < qr_strings_length; i++){
      uint32_t length_st_qr_strings;
      memcpy(&length_st_qr_strings, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_qr_strings; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_qr_strings-1]=0;
      this->st_qr_strings = (char *)(inbuffer + offset-1);
      offset += length_st_qr_strings;
        memcpy( &(this->qr_strings[i]), &(this->st_qr_strings), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return "pr_msgs/QR"; };
    const char * getMD5(){ return "88cf0519306dea0fed4b6d2dec050a5c"; };

  };

}
#endif