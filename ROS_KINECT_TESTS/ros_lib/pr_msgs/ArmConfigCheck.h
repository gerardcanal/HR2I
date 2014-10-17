#ifndef _ROS_SERVICE_ArmConfigCheck_h
#define _ROS_SERVICE_ArmConfigCheck_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/JointState.h"

namespace pr_msgs
{

static const char ARMCONFIGCHECK[] = "pr_msgs/ArmConfigCheck";

  class ArmConfigCheckRequest : public ros::Msg
  {
    public:
      sensor_msgs::JointState joint_state;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->joint_state.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->joint_state.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return ARMCONFIGCHECK; };
    const char * getMD5(){ return "9ca061465ef0ed08771ed240c43789f5"; };

  };

  class ArmConfigCheckResponse : public ros::Msg
  {
    public:
      bool current_self_collision;
      bool current_env_collision;
      bool future_self_collision;
      bool future_env_collision;
      bool current_joint_limits_exceeded;
      bool future_joint_limits_exceeded;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_current_self_collision;
      u_current_self_collision.real = this->current_self_collision;
      *(outbuffer + offset + 0) = (u_current_self_collision.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->current_self_collision);
      union {
        bool real;
        uint8_t base;
      } u_current_env_collision;
      u_current_env_collision.real = this->current_env_collision;
      *(outbuffer + offset + 0) = (u_current_env_collision.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->current_env_collision);
      union {
        bool real;
        uint8_t base;
      } u_future_self_collision;
      u_future_self_collision.real = this->future_self_collision;
      *(outbuffer + offset + 0) = (u_future_self_collision.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->future_self_collision);
      union {
        bool real;
        uint8_t base;
      } u_future_env_collision;
      u_future_env_collision.real = this->future_env_collision;
      *(outbuffer + offset + 0) = (u_future_env_collision.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->future_env_collision);
      union {
        bool real;
        uint8_t base;
      } u_current_joint_limits_exceeded;
      u_current_joint_limits_exceeded.real = this->current_joint_limits_exceeded;
      *(outbuffer + offset + 0) = (u_current_joint_limits_exceeded.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->current_joint_limits_exceeded);
      union {
        bool real;
        uint8_t base;
      } u_future_joint_limits_exceeded;
      u_future_joint_limits_exceeded.real = this->future_joint_limits_exceeded;
      *(outbuffer + offset + 0) = (u_future_joint_limits_exceeded.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->future_joint_limits_exceeded);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_current_self_collision;
      u_current_self_collision.base = 0;
      u_current_self_collision.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->current_self_collision = u_current_self_collision.real;
      offset += sizeof(this->current_self_collision);
      union {
        bool real;
        uint8_t base;
      } u_current_env_collision;
      u_current_env_collision.base = 0;
      u_current_env_collision.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->current_env_collision = u_current_env_collision.real;
      offset += sizeof(this->current_env_collision);
      union {
        bool real;
        uint8_t base;
      } u_future_self_collision;
      u_future_self_collision.base = 0;
      u_future_self_collision.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->future_self_collision = u_future_self_collision.real;
      offset += sizeof(this->future_self_collision);
      union {
        bool real;
        uint8_t base;
      } u_future_env_collision;
      u_future_env_collision.base = 0;
      u_future_env_collision.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->future_env_collision = u_future_env_collision.real;
      offset += sizeof(this->future_env_collision);
      union {
        bool real;
        uint8_t base;
      } u_current_joint_limits_exceeded;
      u_current_joint_limits_exceeded.base = 0;
      u_current_joint_limits_exceeded.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->current_joint_limits_exceeded = u_current_joint_limits_exceeded.real;
      offset += sizeof(this->current_joint_limits_exceeded);
      union {
        bool real;
        uint8_t base;
      } u_future_joint_limits_exceeded;
      u_future_joint_limits_exceeded.base = 0;
      u_future_joint_limits_exceeded.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->future_joint_limits_exceeded = u_future_joint_limits_exceeded.real;
      offset += sizeof(this->future_joint_limits_exceeded);
     return offset;
    }

    const char * getType(){ return ARMCONFIGCHECK; };
    const char * getMD5(){ return "27773a29cfec7a99cc44eea74da93860"; };

  };

  class ArmConfigCheck {
    public:
    typedef ArmConfigCheckRequest Request;
    typedef ArmConfigCheckResponse Response;
  };

}
#endif
