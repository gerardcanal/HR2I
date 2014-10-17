#ifndef _ROS_SERVICE_ResetFinger_h
#define _ROS_SERVICE_ResetFinger_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

static const char RESETFINGER[] = "pr_msgs/ResetFinger";

  class ResetFingerRequest : public ros::Msg
  {
    public:
      int8_t finger;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_finger;
      u_finger.real = this->finger;
      *(outbuffer + offset + 0) = (u_finger.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->finger);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_finger;
      u_finger.base = 0;
      u_finger.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->finger = u_finger.real;
      offset += sizeof(this->finger);
     return offset;
    }

    const char * getType(){ return RESETFINGER; };
    const char * getMD5(){ return "80f9f083fe71012318271dc6232e9766"; };

  };

  class ResetFingerResponse : public ros::Msg
  {
    public:
      bool ok;
      char * reason;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ok;
      u_ok.real = this->ok;
      *(outbuffer + offset + 0) = (u_ok.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ok);
      uint32_t length_reason = strlen( (const char*) this->reason);
      memcpy(outbuffer + offset, &length_reason, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->reason, length_reason);
      offset += length_reason;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ok;
      u_ok.base = 0;
      u_ok.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ok = u_ok.real;
      offset += sizeof(this->ok);
      uint32_t length_reason;
      memcpy(&length_reason, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_reason; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_reason-1]=0;
      this->reason = (char *)(inbuffer + offset-1);
      offset += length_reason;
     return offset;
    }

    const char * getType(){ return RESETFINGER; };
    const char * getMD5(){ return "4679398f882e7cbdea165980d3ec2888"; };

  };

  class ResetFinger {
    public:
    typedef ResetFingerRequest Request;
    typedef ResetFingerResponse Response;
  };

}
#endif
