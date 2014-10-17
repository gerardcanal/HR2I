#ifndef _ROS_pr_msgs_MassProperties_h
#define _ROS_pr_msgs_MassProperties_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

  class MassProperties : public ros::Msg
  {
    public:
      uint8_t link;
      double mass;
      double cog_x;
      double cog_y;
      double cog_z;
      double inertia_xx;
      double inertia_xy;
      double inertia_xz;
      double inertia_yy;
      double inertia_yz;
      double inertia_zz;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->link >> (8 * 0)) & 0xFF;
      offset += sizeof(this->link);
      union {
        double real;
        uint64_t base;
      } u_mass;
      u_mass.real = this->mass;
      *(outbuffer + offset + 0) = (u_mass.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mass.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mass.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mass.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mass.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mass.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mass.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mass.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mass);
      union {
        double real;
        uint64_t base;
      } u_cog_x;
      u_cog_x.real = this->cog_x;
      *(outbuffer + offset + 0) = (u_cog_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cog_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cog_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cog_x.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_cog_x.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_cog_x.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_cog_x.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_cog_x.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->cog_x);
      union {
        double real;
        uint64_t base;
      } u_cog_y;
      u_cog_y.real = this->cog_y;
      *(outbuffer + offset + 0) = (u_cog_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cog_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cog_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cog_y.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_cog_y.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_cog_y.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_cog_y.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_cog_y.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->cog_y);
      union {
        double real;
        uint64_t base;
      } u_cog_z;
      u_cog_z.real = this->cog_z;
      *(outbuffer + offset + 0) = (u_cog_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cog_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cog_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cog_z.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_cog_z.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_cog_z.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_cog_z.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_cog_z.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->cog_z);
      union {
        double real;
        uint64_t base;
      } u_inertia_xx;
      u_inertia_xx.real = this->inertia_xx;
      *(outbuffer + offset + 0) = (u_inertia_xx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_inertia_xx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_inertia_xx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_inertia_xx.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_inertia_xx.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_inertia_xx.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_inertia_xx.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_inertia_xx.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->inertia_xx);
      union {
        double real;
        uint64_t base;
      } u_inertia_xy;
      u_inertia_xy.real = this->inertia_xy;
      *(outbuffer + offset + 0) = (u_inertia_xy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_inertia_xy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_inertia_xy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_inertia_xy.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_inertia_xy.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_inertia_xy.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_inertia_xy.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_inertia_xy.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->inertia_xy);
      union {
        double real;
        uint64_t base;
      } u_inertia_xz;
      u_inertia_xz.real = this->inertia_xz;
      *(outbuffer + offset + 0) = (u_inertia_xz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_inertia_xz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_inertia_xz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_inertia_xz.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_inertia_xz.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_inertia_xz.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_inertia_xz.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_inertia_xz.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->inertia_xz);
      union {
        double real;
        uint64_t base;
      } u_inertia_yy;
      u_inertia_yy.real = this->inertia_yy;
      *(outbuffer + offset + 0) = (u_inertia_yy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_inertia_yy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_inertia_yy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_inertia_yy.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_inertia_yy.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_inertia_yy.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_inertia_yy.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_inertia_yy.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->inertia_yy);
      union {
        double real;
        uint64_t base;
      } u_inertia_yz;
      u_inertia_yz.real = this->inertia_yz;
      *(outbuffer + offset + 0) = (u_inertia_yz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_inertia_yz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_inertia_yz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_inertia_yz.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_inertia_yz.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_inertia_yz.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_inertia_yz.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_inertia_yz.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->inertia_yz);
      union {
        double real;
        uint64_t base;
      } u_inertia_zz;
      u_inertia_zz.real = this->inertia_zz;
      *(outbuffer + offset + 0) = (u_inertia_zz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_inertia_zz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_inertia_zz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_inertia_zz.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_inertia_zz.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_inertia_zz.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_inertia_zz.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_inertia_zz.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->inertia_zz);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->link =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->link);
      union {
        double real;
        uint64_t base;
      } u_mass;
      u_mass.base = 0;
      u_mass.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mass.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mass.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mass.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mass.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mass.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mass.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mass.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mass = u_mass.real;
      offset += sizeof(this->mass);
      union {
        double real;
        uint64_t base;
      } u_cog_x;
      u_cog_x.base = 0;
      u_cog_x.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cog_x.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cog_x.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cog_x.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_cog_x.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_cog_x.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_cog_x.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_cog_x.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->cog_x = u_cog_x.real;
      offset += sizeof(this->cog_x);
      union {
        double real;
        uint64_t base;
      } u_cog_y;
      u_cog_y.base = 0;
      u_cog_y.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cog_y.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cog_y.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cog_y.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_cog_y.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_cog_y.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_cog_y.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_cog_y.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->cog_y = u_cog_y.real;
      offset += sizeof(this->cog_y);
      union {
        double real;
        uint64_t base;
      } u_cog_z;
      u_cog_z.base = 0;
      u_cog_z.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cog_z.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cog_z.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cog_z.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_cog_z.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_cog_z.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_cog_z.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_cog_z.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->cog_z = u_cog_z.real;
      offset += sizeof(this->cog_z);
      union {
        double real;
        uint64_t base;
      } u_inertia_xx;
      u_inertia_xx.base = 0;
      u_inertia_xx.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_inertia_xx.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_inertia_xx.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_inertia_xx.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_inertia_xx.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_inertia_xx.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_inertia_xx.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_inertia_xx.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->inertia_xx = u_inertia_xx.real;
      offset += sizeof(this->inertia_xx);
      union {
        double real;
        uint64_t base;
      } u_inertia_xy;
      u_inertia_xy.base = 0;
      u_inertia_xy.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_inertia_xy.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_inertia_xy.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_inertia_xy.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_inertia_xy.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_inertia_xy.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_inertia_xy.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_inertia_xy.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->inertia_xy = u_inertia_xy.real;
      offset += sizeof(this->inertia_xy);
      union {
        double real;
        uint64_t base;
      } u_inertia_xz;
      u_inertia_xz.base = 0;
      u_inertia_xz.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_inertia_xz.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_inertia_xz.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_inertia_xz.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_inertia_xz.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_inertia_xz.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_inertia_xz.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_inertia_xz.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->inertia_xz = u_inertia_xz.real;
      offset += sizeof(this->inertia_xz);
      union {
        double real;
        uint64_t base;
      } u_inertia_yy;
      u_inertia_yy.base = 0;
      u_inertia_yy.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_inertia_yy.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_inertia_yy.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_inertia_yy.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_inertia_yy.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_inertia_yy.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_inertia_yy.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_inertia_yy.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->inertia_yy = u_inertia_yy.real;
      offset += sizeof(this->inertia_yy);
      union {
        double real;
        uint64_t base;
      } u_inertia_yz;
      u_inertia_yz.base = 0;
      u_inertia_yz.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_inertia_yz.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_inertia_yz.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_inertia_yz.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_inertia_yz.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_inertia_yz.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_inertia_yz.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_inertia_yz.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->inertia_yz = u_inertia_yz.real;
      offset += sizeof(this->inertia_yz);
      union {
        double real;
        uint64_t base;
      } u_inertia_zz;
      u_inertia_zz.base = 0;
      u_inertia_zz.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_inertia_zz.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_inertia_zz.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_inertia_zz.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_inertia_zz.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_inertia_zz.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_inertia_zz.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_inertia_zz.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->inertia_zz = u_inertia_zz.real;
      offset += sizeof(this->inertia_zz);
     return offset;
    }

    const char * getType(){ return "pr_msgs/MassProperties"; };
    const char * getMD5(){ return "6902a26aa992b6613972882349b094c7"; };

  };

}
#endif