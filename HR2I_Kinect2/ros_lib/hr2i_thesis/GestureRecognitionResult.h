#ifndef _ROS_hr2i_thesis_GestureRecognitionResult_h
#define _ROS_hr2i_thesis_GestureRecognitionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"

namespace hr2i_thesis
{

  class GestureRecognitionResult : public ros::Msg
  {
    public:
      std_msgs::Header header;
      int8_t gestureId;
      geometry_msgs::Point ground_point;
      geometry_msgs::Point person_position;
      enum { idHello =  0 };
      enum { idPointAt =  1 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_gestureId;
      u_gestureId.real = this->gestureId;
      *(outbuffer + offset + 0) = (u_gestureId.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gestureId);
      offset += this->ground_point.serialize(outbuffer + offset);
      offset += this->person_position.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_gestureId;
      u_gestureId.base = 0;
      u_gestureId.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gestureId = u_gestureId.real;
      offset += sizeof(this->gestureId);
      offset += this->ground_point.deserialize(inbuffer + offset);
      offset += this->person_position.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "hr2i_thesis/GestureRecognitionResult"; };
    const char * getMD5(){ return "7508312598b09634611c1ab54295f931"; };

  };

}
#endif