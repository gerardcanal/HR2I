#ifndef _ROS_nao_msgs_Bumper_h
#define _ROS_nao_msgs_Bumper_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace nao_msgs
{

  class Bumper : public ros::Msg
  {
    public:
      uint8_t bumper;
      uint8_t state;
      enum { right = 0 };
      enum { left = 1 };
      enum { stateReleased = 0 };
      enum { statePressed = 1 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->bumper >> (8 * 0)) & 0xFF;
      offset += sizeof(this->bumper);
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->bumper =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->bumper);
      this->state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->state);
     return offset;
    }

    const char * getType(){ return "nao_msgs/Bumper"; };
    const char * getMD5(){ return "4d423b2a165337e812d1b5a1cbab6b8d"; };

  };

}
#endif