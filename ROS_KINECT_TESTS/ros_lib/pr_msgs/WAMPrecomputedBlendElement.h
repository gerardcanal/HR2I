#ifndef _ROS_pr_msgs_WAMPrecomputedBlendElement_h
#define _ROS_pr_msgs_WAMPrecomputedBlendElement_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr_msgs/Joints.h"

namespace pr_msgs
{

  class WAMPrecomputedBlendElement : public ros::Msg
  {
    public:
      pr_msgs::Joints start_pos;
      pr_msgs::Joints end_pos;
      double start_time;
      double duration;
      double max_path_velocity;
      double max_path_acceleration;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->start_pos.serialize(outbuffer + offset);
      offset += this->end_pos.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_start_time;
      u_start_time.real = this->start_time;
      *(outbuffer + offset + 0) = (u_start_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_start_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_start_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_start_time.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_start_time.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_start_time.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_start_time.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_start_time.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->start_time);
      union {
        double real;
        uint64_t base;
      } u_duration;
      u_duration.real = this->duration;
      *(outbuffer + offset + 0) = (u_duration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_duration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_duration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_duration.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_duration.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_duration.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_duration.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_duration.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->duration);
      union {
        double real;
        uint64_t base;
      } u_max_path_velocity;
      u_max_path_velocity.real = this->max_path_velocity;
      *(outbuffer + offset + 0) = (u_max_path_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_path_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_path_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_path_velocity.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_path_velocity.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_path_velocity.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_path_velocity.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_path_velocity.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_path_velocity);
      union {
        double real;
        uint64_t base;
      } u_max_path_acceleration;
      u_max_path_acceleration.real = this->max_path_acceleration;
      *(outbuffer + offset + 0) = (u_max_path_acceleration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_path_acceleration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_path_acceleration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_path_acceleration.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_path_acceleration.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_path_acceleration.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_path_acceleration.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_path_acceleration.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_path_acceleration);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->start_pos.deserialize(inbuffer + offset);
      offset += this->end_pos.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_start_time;
      u_start_time.base = 0;
      u_start_time.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_start_time.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_start_time.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_start_time.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_start_time.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_start_time.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_start_time.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_start_time.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->start_time = u_start_time.real;
      offset += sizeof(this->start_time);
      union {
        double real;
        uint64_t base;
      } u_duration;
      u_duration.base = 0;
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->duration = u_duration.real;
      offset += sizeof(this->duration);
      union {
        double real;
        uint64_t base;
      } u_max_path_velocity;
      u_max_path_velocity.base = 0;
      u_max_path_velocity.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_path_velocity.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_path_velocity.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_path_velocity.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_max_path_velocity.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_max_path_velocity.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_max_path_velocity.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_max_path_velocity.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->max_path_velocity = u_max_path_velocity.real;
      offset += sizeof(this->max_path_velocity);
      union {
        double real;
        uint64_t base;
      } u_max_path_acceleration;
      u_max_path_acceleration.base = 0;
      u_max_path_acceleration.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_path_acceleration.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_path_acceleration.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_path_acceleration.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_max_path_acceleration.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_max_path_acceleration.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_max_path_acceleration.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_max_path_acceleration.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->max_path_acceleration = u_max_path_acceleration.real;
      offset += sizeof(this->max_path_acceleration);
     return offset;
    }

    const char * getType(){ return "pr_msgs/WAMPrecomputedBlendElement"; };
    const char * getMD5(){ return "431cf004c67a8db19f0c5e2f55655018"; };

  };

}
#endif