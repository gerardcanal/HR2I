#ifndef _ROS_SERVICE_MoveToService_h
#define _ROS_SERVICE_MoveToService_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose2D.h"

namespace wifibot_utils
{

static const char MOVETOSERVICE[] = "wifibot_utils/MoveToService";

  class MoveToServiceRequest : public ros::Msg
  {
    public:
      geometry_msgs::Pose2D pose;
      bool orient;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pose.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_orient;
      u_orient.real = this->orient;
      *(outbuffer + offset + 0) = (u_orient.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->orient);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pose.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_orient;
      u_orient.base = 0;
      u_orient.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->orient = u_orient.real;
      offset += sizeof(this->orient);
     return offset;
    }

    const char * getType(){ return MOVETOSERVICE; };
    const char * getMD5(){ return "844f5f2b305ed6f7fd6639c9b1efdc0b"; };

  };

  class MoveToServiceResponse : public ros::Msg
  {
    public:
      bool result;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.real = this->result;
      *(outbuffer + offset + 0) = (u_result.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.base = 0;
      u_result.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->result = u_result.real;
      offset += sizeof(this->result);
     return offset;
    }

    const char * getType(){ return MOVETOSERVICE; };
    const char * getMD5(){ return "eb13ac1f1354ccecb7941ee8fa2192e8"; };

  };

  class MoveToService {
    public:
    typedef MoveToServiceRequest Request;
    typedef MoveToServiceResponse Response;
  };

}
#endif
