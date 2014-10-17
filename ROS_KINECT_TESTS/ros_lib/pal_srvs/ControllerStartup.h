#ifndef _ROS_SERVICE_ControllerStartup_h
#define _ROS_SERVICE_ControllerStartup_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pal_srvs
{

static const char CONTROLLERSTARTUP[] = "pal_srvs/ControllerStartup";

  class ControllerStartupRequest : public ros::Msg
  {
    public:
      char * Controller;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_Controller = strlen( (const char*) this->Controller);
      memcpy(outbuffer + offset, &length_Controller, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->Controller, length_Controller);
      offset += length_Controller;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_Controller;
      memcpy(&length_Controller, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_Controller; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_Controller-1]=0;
      this->Controller = (char *)(inbuffer + offset-1);
      offset += length_Controller;
     return offset;
    }

    const char * getType(){ return CONTROLLERSTARTUP; };
    const char * getMD5(){ return "9f70e99f79dc86b0a22a277368fb4080"; };

  };

  class ControllerStartupResponse : public ros::Msg
  {
    public:
      bool Result;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_Result;
      u_Result.real = this->Result;
      *(outbuffer + offset + 0) = (u_Result.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Result);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_Result;
      u_Result.base = 0;
      u_Result.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->Result = u_Result.real;
      offset += sizeof(this->Result);
     return offset;
    }

    const char * getType(){ return CONTROLLERSTARTUP; };
    const char * getMD5(){ return "85ed39ee8c4e8f1c21743e6fe4dd523e"; };

  };

  class ControllerStartup {
    public:
    typedef ControllerStartupRequest Request;
    typedef ControllerStartupResponse Response;
  };

}
#endif
