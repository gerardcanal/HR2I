#ifndef _ROS_pr_msgs_Vector3D_h
#define _ROS_pr_msgs_Vector3D_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

  class Vector3D : public ros::Msg
  {
    public:
      double axis1;
      double axis2;
      double axis3;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_axis1;
      u_axis1.real = this->axis1;
      *(outbuffer + offset + 0) = (u_axis1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_axis1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_axis1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_axis1.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_axis1.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_axis1.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_axis1.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_axis1.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->axis1);
      union {
        double real;
        uint64_t base;
      } u_axis2;
      u_axis2.real = this->axis2;
      *(outbuffer + offset + 0) = (u_axis2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_axis2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_axis2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_axis2.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_axis2.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_axis2.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_axis2.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_axis2.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->axis2);
      union {
        double real;
        uint64_t base;
      } u_axis3;
      u_axis3.real = this->axis3;
      *(outbuffer + offset + 0) = (u_axis3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_axis3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_axis3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_axis3.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_axis3.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_axis3.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_axis3.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_axis3.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->axis3);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_axis1;
      u_axis1.base = 0;
      u_axis1.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_axis1.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_axis1.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_axis1.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_axis1.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_axis1.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_axis1.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_axis1.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->axis1 = u_axis1.real;
      offset += sizeof(this->axis1);
      union {
        double real;
        uint64_t base;
      } u_axis2;
      u_axis2.base = 0;
      u_axis2.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_axis2.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_axis2.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_axis2.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_axis2.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_axis2.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_axis2.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_axis2.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->axis2 = u_axis2.real;
      offset += sizeof(this->axis2);
      union {
        double real;
        uint64_t base;
      } u_axis3;
      u_axis3.base = 0;
      u_axis3.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_axis3.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_axis3.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_axis3.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_axis3.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_axis3.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_axis3.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_axis3.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->axis3 = u_axis3.real;
      offset += sizeof(this->axis3);
     return offset;
    }

    const char * getType(){ return "pr_msgs/Vector3D"; };
    const char * getMD5(){ return "ebd79d666ce8461ba522577a93648fee"; };

  };

}
#endif