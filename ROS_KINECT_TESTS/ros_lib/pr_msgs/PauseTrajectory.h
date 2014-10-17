#ifndef _ROS_SERVICE_PauseTrajectory_h
#define _ROS_SERVICE_PauseTrajectory_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

static const char PAUSETRAJECTORY[] = "pr_msgs/PauseTrajectory";

  class PauseTrajectoryRequest : public ros::Msg
  {
    public:
      int8_t pause;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_pause;
      u_pause.real = this->pause;
      *(outbuffer + offset + 0) = (u_pause.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pause);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_pause;
      u_pause.base = 0;
      u_pause.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->pause = u_pause.real;
      offset += sizeof(this->pause);
     return offset;
    }

    const char * getType(){ return PAUSETRAJECTORY; };
    const char * getMD5(){ return "0151c2e2197311074b70533572da3f44"; };

  };

  class PauseTrajectoryResponse : public ros::Msg
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

    const char * getType(){ return PAUSETRAJECTORY; };
    const char * getMD5(){ return "4679398f882e7cbdea165980d3ec2888"; };

  };

  class PauseTrajectory {
    public:
    typedef PauseTrajectoryRequest Request;
    typedef PauseTrajectoryResponse Response;
  };

}
#endif
