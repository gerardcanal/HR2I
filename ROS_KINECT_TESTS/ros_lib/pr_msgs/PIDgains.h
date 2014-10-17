#ifndef _ROS_pr_msgs_PIDgains_h
#define _ROS_pr_msgs_PIDgains_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

  class PIDgains : public ros::Msg
  {
    public:
      double kp;
      double kd;
      double ki;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_kp;
      u_kp.real = this->kp;
      *(outbuffer + offset + 0) = (u_kp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kp.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_kp.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_kp.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_kp.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_kp.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->kp);
      union {
        double real;
        uint64_t base;
      } u_kd;
      u_kd.real = this->kd;
      *(outbuffer + offset + 0) = (u_kd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kd.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_kd.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_kd.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_kd.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_kd.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->kd);
      union {
        double real;
        uint64_t base;
      } u_ki;
      u_ki.real = this->ki;
      *(outbuffer + offset + 0) = (u_ki.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ki.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ki.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ki.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_ki.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_ki.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_ki.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_ki.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->ki);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_kp;
      u_kp.base = 0;
      u_kp.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kp.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kp.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kp.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_kp.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_kp.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_kp.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_kp.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->kp = u_kp.real;
      offset += sizeof(this->kp);
      union {
        double real;
        uint64_t base;
      } u_kd;
      u_kd.base = 0;
      u_kd.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kd.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kd.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kd.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_kd.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_kd.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_kd.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_kd.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->kd = u_kd.real;
      offset += sizeof(this->kd);
      union {
        double real;
        uint64_t base;
      } u_ki;
      u_ki.base = 0;
      u_ki.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ki.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ki.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ki.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_ki.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_ki.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_ki.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_ki.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->ki = u_ki.real;
      offset += sizeof(this->ki);
     return offset;
    }

    const char * getType(){ return "pr_msgs/PIDgains"; };
    const char * getMD5(){ return "e87782bd839ad4d19186c03a29ec1538"; };

  };

}
#endif