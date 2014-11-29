#ifndef _ROS_pr_msgs_Servo_h
#define _ROS_pr_msgs_Servo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

  class Servo : public ros::Msg
  {
    public:
      uint8_t joint_length;
      uint32_t st_joint;
      uint32_t * joint;
      uint8_t velocity_length;
      float st_velocity;
      float * velocity;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = joint_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < joint_length; i++){
      *(outbuffer + offset + 0) = (this->joint[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint[i]);
      }
      *(outbuffer + offset++) = velocity_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < velocity_length; i++){
      union {
        float real;
        uint32_t base;
      } u_velocityi;
      u_velocityi.real = this->velocity[i];
      *(outbuffer + offset + 0) = (u_velocityi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocityi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocityi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocityi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t joint_lengthT = *(inbuffer + offset++);
      if(joint_lengthT > joint_length)
        this->joint = (uint32_t*)realloc(this->joint, joint_lengthT * sizeof(uint32_t));
      offset += 3;
      joint_length = joint_lengthT;
      for( uint8_t i = 0; i < joint_length; i++){
      this->st_joint =  ((uint32_t) (*(inbuffer + offset)));
      this->st_joint |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_joint |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_joint |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_joint);
        memcpy( &(this->joint[i]), &(this->st_joint), sizeof(uint32_t));
      }
      uint8_t velocity_lengthT = *(inbuffer + offset++);
      if(velocity_lengthT > velocity_length)
        this->velocity = (float*)realloc(this->velocity, velocity_lengthT * sizeof(float));
      offset += 3;
      velocity_length = velocity_lengthT;
      for( uint8_t i = 0; i < velocity_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_velocity;
      u_st_velocity.base = 0;
      u_st_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_velocity = u_st_velocity.real;
      offset += sizeof(this->st_velocity);
        memcpy( &(this->velocity[i]), &(this->st_velocity), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "pr_msgs/Servo"; };
    const char * getMD5(){ return "86bc1137fbe8eaaaaa9dbbe92ec4bad5"; };

  };

}
#endif