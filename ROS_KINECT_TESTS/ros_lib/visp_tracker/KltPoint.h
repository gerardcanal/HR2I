#ifndef _ROS_visp_tracker_KltPoint_h
#define _ROS_visp_tracker_KltPoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace visp_tracker
{

  class KltPoint : public ros::Msg
  {
    public:
      double i;
      double j;
      int32_t id;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_i;
      u_i.real = this->i;
      *(outbuffer + offset + 0) = (u_i.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_i.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_i.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_i.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_i.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_i.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_i.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_i.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->i);
      union {
        double real;
        uint64_t base;
      } u_j;
      u_j.real = this->j;
      *(outbuffer + offset + 0) = (u_j.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_j.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_j.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_j.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_j.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_j.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_j.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_j.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->j);
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_i;
      u_i.base = 0;
      u_i.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_i.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_i.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_i.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_i.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_i.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_i.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_i.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->i = u_i.real;
      offset += sizeof(this->i);
      union {
        double real;
        uint64_t base;
      } u_j;
      u_j.base = 0;
      u_j.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_j.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_j.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_j.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_j.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_j.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_j.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_j.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->j = u_j.real;
      offset += sizeof(this->j);
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->id = u_id.real;
      offset += sizeof(this->id);
     return offset;
    }

    const char * getType(){ return "visp_tracker/KltPoint"; };
    const char * getMD5(){ return "6014cf727908e3e35d0048962e94796f"; };

  };

}
#endif