#ifndef _ROS_pr_msgs_HandOff_h
#define _ROS_pr_msgs_HandOff_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"

namespace pr_msgs
{

  class HandOff : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::Point point;
      geometry_msgs::Point point_world;
      geometry_msgs::Point offset;
      int16_t status;
      int16_t mode;
      int16_t object;
      enum { status_none = 0 };
      enum { status_handoff = 1 };
      enum { mode_retract = 0 };
      enum { mode_follow = 1 };
      enum { mode_advance = 2 };
      enum { object_none = 0 };
      enum { object_poptarts = 1 };
      enum { object_fuze = 2 };
      enum { object_container = 3 };
      enum { object_unknown = 4 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->point.serialize(outbuffer + offset);
      offset += this->point_world.serialize(outbuffer + offset);
      offset += this->offset.serialize(outbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_status.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->status);
      union {
        int16_t real;
        uint16_t base;
      } u_mode;
      u_mode.real = this->mode;
      *(outbuffer + offset + 0) = (u_mode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mode.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->mode);
      union {
        int16_t real;
        uint16_t base;
      } u_object;
      u_object.real = this->object;
      *(outbuffer + offset + 0) = (u_object.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_object.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->object);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->point.deserialize(inbuffer + offset);
      offset += this->point_world.deserialize(inbuffer + offset);
      offset += this->offset.deserialize(inbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_status.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->status = u_status.real;
      offset += sizeof(this->status);
      union {
        int16_t real;
        uint16_t base;
      } u_mode;
      u_mode.base = 0;
      u_mode.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mode.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->mode = u_mode.real;
      offset += sizeof(this->mode);
      union {
        int16_t real;
        uint16_t base;
      } u_object;
      u_object.base = 0;
      u_object.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_object.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->object = u_object.real;
      offset += sizeof(this->object);
     return offset;
    }

    const char * getType(){ return "pr_msgs/HandOff"; };
    const char * getMD5(){ return "8d06d08893b771d81e1543bd706af0bf"; };

  };

}
#endif