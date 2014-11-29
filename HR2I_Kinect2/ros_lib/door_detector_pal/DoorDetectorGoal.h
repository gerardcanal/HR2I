#ifndef _ROS_door_detector_pal_DoorDetectorGoal_h
#define _ROS_door_detector_pal_DoorDetectorGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace door_detector_pal
{

  class DoorDetectorGoal : public ros::Msg
  {
    public:
      int8_t votation;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_votation;
      u_votation.real = this->votation;
      *(outbuffer + offset + 0) = (u_votation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->votation);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_votation;
      u_votation.base = 0;
      u_votation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->votation = u_votation.real;
      offset += sizeof(this->votation);
     return offset;
    }

    const char * getType(){ return "door_detector_pal/DoorDetectorGoal"; };
    const char * getMD5(){ return "33dde40a30662752291cd32efdcae2f5"; };

  };

}
#endif