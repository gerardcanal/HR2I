#ifndef _ROS_naoqi_msgs_TactileTouch_h
#define _ROS_naoqi_msgs_TactileTouch_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace naoqi_msgs
{

  class TactileTouch : public ros::Msg
  {
    public:
      uint8_t button;
      uint8_t state;
      enum { buttonFront = 1 };
      enum { buttonMiddle = 2 };
      enum { buttonRear = 3 };
      enum { stateReleased = 0 };
      enum { statePressed = 1 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->button >> (8 * 0)) & 0xFF;
      offset += sizeof(this->button);
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->button =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->button);
      this->state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->state);
     return offset;
    }

    const char * getType(){ return "naoqi_msgs/TactileTouch"; };
    const char * getMD5(){ return "b75165bf9dfed26d50ad4e3162304225"; };

  };

}
#endif