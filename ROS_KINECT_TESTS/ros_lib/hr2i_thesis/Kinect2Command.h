#ifndef _ROS_hr2i_thesis_Kinect2Command_h
#define _ROS_hr2i_thesis_Kinect2Command_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hr2i_thesis
{

  class Kinect2Command : public ros::Msg
  {
    public:
      int8_t command;
      enum { recGestCmd =  0 };
      enum { segmentBlobs =  1 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_command;
      u_command.real = this->command;
      *(outbuffer + offset + 0) = (u_command.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->command);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_command;
      u_command.base = 0;
      u_command.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->command = u_command.real;
      offset += sizeof(this->command);
     return offset;
    }

    const char * getType(){ return "hr2i_thesis/Kinect2Command"; };
    const char * getMD5(){ return "165f44d2b02d95eedf6bc610f1438c91"; };

  };

}
#endif