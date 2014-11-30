#ifndef _ROS_roswifibot_WStatus_h
#define _ROS_roswifibot_WStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace roswifibot
{

  class WStatus : public ros::Msg
  {
    public:
      double battery_level;
      double current;
      int32_t ADC1;
      int32_t ADC2;
      int32_t ADC3;
      int32_t ADC4;
      double speed_front_left;
      double speed_front_right;
      double odometry_left;
      double odometry_right;
      int32_t version;
      int32_t relay1;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_battery_level;
      u_battery_level.real = this->battery_level;
      *(outbuffer + offset + 0) = (u_battery_level.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_level.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_level.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_level.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_battery_level.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_battery_level.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_battery_level.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_battery_level.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->battery_level);
      union {
        double real;
        uint64_t base;
      } u_current;
      u_current.real = this->current;
      *(outbuffer + offset + 0) = (u_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_current.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_current.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_current.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_current.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->current);
      union {
        int32_t real;
        uint32_t base;
      } u_ADC1;
      u_ADC1.real = this->ADC1;
      *(outbuffer + offset + 0) = (u_ADC1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ADC1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ADC1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ADC1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ADC1);
      union {
        int32_t real;
        uint32_t base;
      } u_ADC2;
      u_ADC2.real = this->ADC2;
      *(outbuffer + offset + 0) = (u_ADC2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ADC2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ADC2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ADC2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ADC2);
      union {
        int32_t real;
        uint32_t base;
      } u_ADC3;
      u_ADC3.real = this->ADC3;
      *(outbuffer + offset + 0) = (u_ADC3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ADC3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ADC3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ADC3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ADC3);
      union {
        int32_t real;
        uint32_t base;
      } u_ADC4;
      u_ADC4.real = this->ADC4;
      *(outbuffer + offset + 0) = (u_ADC4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ADC4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ADC4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ADC4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ADC4);
      union {
        double real;
        uint64_t base;
      } u_speed_front_left;
      u_speed_front_left.real = this->speed_front_left;
      *(outbuffer + offset + 0) = (u_speed_front_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_front_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_front_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_front_left.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_speed_front_left.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_speed_front_left.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_speed_front_left.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_speed_front_left.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->speed_front_left);
      union {
        double real;
        uint64_t base;
      } u_speed_front_right;
      u_speed_front_right.real = this->speed_front_right;
      *(outbuffer + offset + 0) = (u_speed_front_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_front_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_front_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_front_right.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_speed_front_right.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_speed_front_right.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_speed_front_right.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_speed_front_right.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->speed_front_right);
      union {
        double real;
        uint64_t base;
      } u_odometry_left;
      u_odometry_left.real = this->odometry_left;
      *(outbuffer + offset + 0) = (u_odometry_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_odometry_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_odometry_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_odometry_left.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_odometry_left.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_odometry_left.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_odometry_left.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_odometry_left.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->odometry_left);
      union {
        double real;
        uint64_t base;
      } u_odometry_right;
      u_odometry_right.real = this->odometry_right;
      *(outbuffer + offset + 0) = (u_odometry_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_odometry_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_odometry_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_odometry_right.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_odometry_right.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_odometry_right.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_odometry_right.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_odometry_right.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->odometry_right);
      union {
        int32_t real;
        uint32_t base;
      } u_version;
      u_version.real = this->version;
      *(outbuffer + offset + 0) = (u_version.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_version.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_version.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_version.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->version);
      union {
        int32_t real;
        uint32_t base;
      } u_relay1;
      u_relay1.real = this->relay1;
      *(outbuffer + offset + 0) = (u_relay1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_relay1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_relay1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_relay1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->relay1);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_battery_level;
      u_battery_level.base = 0;
      u_battery_level.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_level.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_level.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_level.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_battery_level.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_battery_level.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_battery_level.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_battery_level.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->battery_level = u_battery_level.real;
      offset += sizeof(this->battery_level);
      union {
        double real;
        uint64_t base;
      } u_current;
      u_current.base = 0;
      u_current.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_current.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_current.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_current.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_current.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->current = u_current.real;
      offset += sizeof(this->current);
      union {
        int32_t real;
        uint32_t base;
      } u_ADC1;
      u_ADC1.base = 0;
      u_ADC1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ADC1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ADC1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ADC1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ADC1 = u_ADC1.real;
      offset += sizeof(this->ADC1);
      union {
        int32_t real;
        uint32_t base;
      } u_ADC2;
      u_ADC2.base = 0;
      u_ADC2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ADC2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ADC2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ADC2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ADC2 = u_ADC2.real;
      offset += sizeof(this->ADC2);
      union {
        int32_t real;
        uint32_t base;
      } u_ADC3;
      u_ADC3.base = 0;
      u_ADC3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ADC3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ADC3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ADC3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ADC3 = u_ADC3.real;
      offset += sizeof(this->ADC3);
      union {
        int32_t real;
        uint32_t base;
      } u_ADC4;
      u_ADC4.base = 0;
      u_ADC4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ADC4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ADC4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ADC4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ADC4 = u_ADC4.real;
      offset += sizeof(this->ADC4);
      union {
        double real;
        uint64_t base;
      } u_speed_front_left;
      u_speed_front_left.base = 0;
      u_speed_front_left.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_front_left.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_front_left.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_front_left.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_speed_front_left.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_speed_front_left.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_speed_front_left.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_speed_front_left.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->speed_front_left = u_speed_front_left.real;
      offset += sizeof(this->speed_front_left);
      union {
        double real;
        uint64_t base;
      } u_speed_front_right;
      u_speed_front_right.base = 0;
      u_speed_front_right.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_front_right.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_front_right.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_front_right.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_speed_front_right.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_speed_front_right.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_speed_front_right.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_speed_front_right.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->speed_front_right = u_speed_front_right.real;
      offset += sizeof(this->speed_front_right);
      union {
        double real;
        uint64_t base;
      } u_odometry_left;
      u_odometry_left.base = 0;
      u_odometry_left.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_odometry_left.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_odometry_left.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_odometry_left.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_odometry_left.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_odometry_left.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_odometry_left.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_odometry_left.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->odometry_left = u_odometry_left.real;
      offset += sizeof(this->odometry_left);
      union {
        double real;
        uint64_t base;
      } u_odometry_right;
      u_odometry_right.base = 0;
      u_odometry_right.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_odometry_right.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_odometry_right.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_odometry_right.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_odometry_right.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_odometry_right.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_odometry_right.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_odometry_right.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->odometry_right = u_odometry_right.real;
      offset += sizeof(this->odometry_right);
      union {
        int32_t real;
        uint32_t base;
      } u_version;
      u_version.base = 0;
      u_version.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_version.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_version.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_version.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->version = u_version.real;
      offset += sizeof(this->version);
      union {
        int32_t real;
        uint32_t base;
      } u_relay1;
      u_relay1.base = 0;
      u_relay1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_relay1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_relay1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_relay1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->relay1 = u_relay1.real;
      offset += sizeof(this->relay1);
     return offset;
    }

    const char * getType(){ return "roswifibot/WStatus"; };
    const char * getMD5(){ return "37bcaf258748a50f0e114a698e6097e9"; };

  };

}
#endif