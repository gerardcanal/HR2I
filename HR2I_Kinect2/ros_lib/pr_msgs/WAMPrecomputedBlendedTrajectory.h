#ifndef _ROS_pr_msgs_WAMPrecomputedBlendedTrajectory_h
#define _ROS_pr_msgs_WAMPrecomputedBlendedTrajectory_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr_msgs/Joints.h"
#include "pr_msgs/WAMPrecomputedBlendElement.h"

namespace pr_msgs
{

  class WAMPrecomputedBlendedTrajectory : public ros::Msg
  {
    public:
      int16_t id;
      bool HoldOnStall;
      bool WaitForStart;
      pr_msgs::Joints start_position;
      pr_msgs::Joints end_position;
      pr_msgs::Joints max_joint_vel;
      pr_msgs::Joints max_joint_accel;
      uint8_t macpieces_length;
      pr_msgs::WAMPrecomputedBlendElement st_macpieces;
      pr_msgs::WAMPrecomputedBlendElement * macpieces;
      double traj_duration;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->id);
      union {
        bool real;
        uint8_t base;
      } u_HoldOnStall;
      u_HoldOnStall.real = this->HoldOnStall;
      *(outbuffer + offset + 0) = (u_HoldOnStall.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->HoldOnStall);
      union {
        bool real;
        uint8_t base;
      } u_WaitForStart;
      u_WaitForStart.real = this->WaitForStart;
      *(outbuffer + offset + 0) = (u_WaitForStart.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->WaitForStart);
      offset += this->start_position.serialize(outbuffer + offset);
      offset += this->end_position.serialize(outbuffer + offset);
      offset += this->max_joint_vel.serialize(outbuffer + offset);
      offset += this->max_joint_accel.serialize(outbuffer + offset);
      *(outbuffer + offset++) = macpieces_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < macpieces_length; i++){
      offset += this->macpieces[i].serialize(outbuffer + offset);
      }
      union {
        double real;
        uint64_t base;
      } u_traj_duration;
      u_traj_duration.real = this->traj_duration;
      *(outbuffer + offset + 0) = (u_traj_duration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_traj_duration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_traj_duration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_traj_duration.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_traj_duration.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_traj_duration.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_traj_duration.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_traj_duration.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->traj_duration);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->id = u_id.real;
      offset += sizeof(this->id);
      union {
        bool real;
        uint8_t base;
      } u_HoldOnStall;
      u_HoldOnStall.base = 0;
      u_HoldOnStall.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->HoldOnStall = u_HoldOnStall.real;
      offset += sizeof(this->HoldOnStall);
      union {
        bool real;
        uint8_t base;
      } u_WaitForStart;
      u_WaitForStart.base = 0;
      u_WaitForStart.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->WaitForStart = u_WaitForStart.real;
      offset += sizeof(this->WaitForStart);
      offset += this->start_position.deserialize(inbuffer + offset);
      offset += this->end_position.deserialize(inbuffer + offset);
      offset += this->max_joint_vel.deserialize(inbuffer + offset);
      offset += this->max_joint_accel.deserialize(inbuffer + offset);
      uint8_t macpieces_lengthT = *(inbuffer + offset++);
      if(macpieces_lengthT > macpieces_length)
        this->macpieces = (pr_msgs::WAMPrecomputedBlendElement*)realloc(this->macpieces, macpieces_lengthT * sizeof(pr_msgs::WAMPrecomputedBlendElement));
      offset += 3;
      macpieces_length = macpieces_lengthT;
      for( uint8_t i = 0; i < macpieces_length; i++){
      offset += this->st_macpieces.deserialize(inbuffer + offset);
        memcpy( &(this->macpieces[i]), &(this->st_macpieces), sizeof(pr_msgs::WAMPrecomputedBlendElement));
      }
      union {
        double real;
        uint64_t base;
      } u_traj_duration;
      u_traj_duration.base = 0;
      u_traj_duration.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_traj_duration.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_traj_duration.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_traj_duration.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_traj_duration.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_traj_duration.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_traj_duration.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_traj_duration.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->traj_duration = u_traj_duration.real;
      offset += sizeof(this->traj_duration);
     return offset;
    }

    const char * getType(){ return "pr_msgs/WAMPrecomputedBlendedTrajectory"; };
    const char * getMD5(){ return "71bcabe3695718cad854012f233bf235"; };

  };

}
#endif