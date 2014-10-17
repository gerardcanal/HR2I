#ifndef _ROS_pr_msgs_WAMInternals_h
#define _ROS_pr_msgs_WAMInternals_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr_msgs/PIDgains.h"

namespace pr_msgs
{

  class WAMInternals : public ros::Msg
  {
    public:
      uint8_t positions_length;
      double st_positions;
      double * positions;
      uint8_t total_torque_length;
      double st_total_torque;
      double * total_torque;
      uint8_t dynamic_torque_length;
      double st_dynamic_torque;
      double * dynamic_torque;
      uint8_t trajectory_torque_length;
      double st_trajectory_torque;
      double * trajectory_torque;
      uint8_t sim_torque_length;
      double st_sim_torque;
      double * sim_torque;
      uint8_t gains_length;
      pr_msgs::PIDgains st_gains;
      pr_msgs::PIDgains * gains;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = positions_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < positions_length; i++){
      union {
        double real;
        uint64_t base;
      } u_positionsi;
      u_positionsi.real = this->positions[i];
      *(outbuffer + offset + 0) = (u_positionsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_positionsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_positionsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_positionsi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_positionsi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_positionsi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_positionsi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_positionsi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->positions[i]);
      }
      *(outbuffer + offset++) = total_torque_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < total_torque_length; i++){
      union {
        double real;
        uint64_t base;
      } u_total_torquei;
      u_total_torquei.real = this->total_torque[i];
      *(outbuffer + offset + 0) = (u_total_torquei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_total_torquei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_total_torquei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_total_torquei.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_total_torquei.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_total_torquei.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_total_torquei.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_total_torquei.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->total_torque[i]);
      }
      *(outbuffer + offset++) = dynamic_torque_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < dynamic_torque_length; i++){
      union {
        double real;
        uint64_t base;
      } u_dynamic_torquei;
      u_dynamic_torquei.real = this->dynamic_torque[i];
      *(outbuffer + offset + 0) = (u_dynamic_torquei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dynamic_torquei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dynamic_torquei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dynamic_torquei.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_dynamic_torquei.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_dynamic_torquei.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_dynamic_torquei.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_dynamic_torquei.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->dynamic_torque[i]);
      }
      *(outbuffer + offset++) = trajectory_torque_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < trajectory_torque_length; i++){
      union {
        double real;
        uint64_t base;
      } u_trajectory_torquei;
      u_trajectory_torquei.real = this->trajectory_torque[i];
      *(outbuffer + offset + 0) = (u_trajectory_torquei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_trajectory_torquei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_trajectory_torquei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_trajectory_torquei.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_trajectory_torquei.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_trajectory_torquei.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_trajectory_torquei.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_trajectory_torquei.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->trajectory_torque[i]);
      }
      *(outbuffer + offset++) = sim_torque_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < sim_torque_length; i++){
      union {
        double real;
        uint64_t base;
      } u_sim_torquei;
      u_sim_torquei.real = this->sim_torque[i];
      *(outbuffer + offset + 0) = (u_sim_torquei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sim_torquei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sim_torquei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sim_torquei.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_sim_torquei.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_sim_torquei.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_sim_torquei.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_sim_torquei.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->sim_torque[i]);
      }
      *(outbuffer + offset++) = gains_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < gains_length; i++){
      offset += this->gains[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t positions_lengthT = *(inbuffer + offset++);
      if(positions_lengthT > positions_length)
        this->positions = (double*)realloc(this->positions, positions_lengthT * sizeof(double));
      offset += 3;
      positions_length = positions_lengthT;
      for( uint8_t i = 0; i < positions_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_positions;
      u_st_positions.base = 0;
      u_st_positions.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_positions.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_positions.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_positions.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_positions.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_positions.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_positions.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_positions.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_positions = u_st_positions.real;
      offset += sizeof(this->st_positions);
        memcpy( &(this->positions[i]), &(this->st_positions), sizeof(double));
      }
      uint8_t total_torque_lengthT = *(inbuffer + offset++);
      if(total_torque_lengthT > total_torque_length)
        this->total_torque = (double*)realloc(this->total_torque, total_torque_lengthT * sizeof(double));
      offset += 3;
      total_torque_length = total_torque_lengthT;
      for( uint8_t i = 0; i < total_torque_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_total_torque;
      u_st_total_torque.base = 0;
      u_st_total_torque.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_total_torque.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_total_torque.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_total_torque.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_total_torque.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_total_torque.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_total_torque.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_total_torque.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_total_torque = u_st_total_torque.real;
      offset += sizeof(this->st_total_torque);
        memcpy( &(this->total_torque[i]), &(this->st_total_torque), sizeof(double));
      }
      uint8_t dynamic_torque_lengthT = *(inbuffer + offset++);
      if(dynamic_torque_lengthT > dynamic_torque_length)
        this->dynamic_torque = (double*)realloc(this->dynamic_torque, dynamic_torque_lengthT * sizeof(double));
      offset += 3;
      dynamic_torque_length = dynamic_torque_lengthT;
      for( uint8_t i = 0; i < dynamic_torque_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_dynamic_torque;
      u_st_dynamic_torque.base = 0;
      u_st_dynamic_torque.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_dynamic_torque.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_dynamic_torque.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_dynamic_torque.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_dynamic_torque.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_dynamic_torque.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_dynamic_torque.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_dynamic_torque.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_dynamic_torque = u_st_dynamic_torque.real;
      offset += sizeof(this->st_dynamic_torque);
        memcpy( &(this->dynamic_torque[i]), &(this->st_dynamic_torque), sizeof(double));
      }
      uint8_t trajectory_torque_lengthT = *(inbuffer + offset++);
      if(trajectory_torque_lengthT > trajectory_torque_length)
        this->trajectory_torque = (double*)realloc(this->trajectory_torque, trajectory_torque_lengthT * sizeof(double));
      offset += 3;
      trajectory_torque_length = trajectory_torque_lengthT;
      for( uint8_t i = 0; i < trajectory_torque_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_trajectory_torque;
      u_st_trajectory_torque.base = 0;
      u_st_trajectory_torque.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_trajectory_torque.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_trajectory_torque.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_trajectory_torque.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_trajectory_torque.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_trajectory_torque.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_trajectory_torque.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_trajectory_torque.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_trajectory_torque = u_st_trajectory_torque.real;
      offset += sizeof(this->st_trajectory_torque);
        memcpy( &(this->trajectory_torque[i]), &(this->st_trajectory_torque), sizeof(double));
      }
      uint8_t sim_torque_lengthT = *(inbuffer + offset++);
      if(sim_torque_lengthT > sim_torque_length)
        this->sim_torque = (double*)realloc(this->sim_torque, sim_torque_lengthT * sizeof(double));
      offset += 3;
      sim_torque_length = sim_torque_lengthT;
      for( uint8_t i = 0; i < sim_torque_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_sim_torque;
      u_st_sim_torque.base = 0;
      u_st_sim_torque.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_sim_torque.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_sim_torque.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_sim_torque.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_sim_torque.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_sim_torque.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_sim_torque.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_sim_torque.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_sim_torque = u_st_sim_torque.real;
      offset += sizeof(this->st_sim_torque);
        memcpy( &(this->sim_torque[i]), &(this->st_sim_torque), sizeof(double));
      }
      uint8_t gains_lengthT = *(inbuffer + offset++);
      if(gains_lengthT > gains_length)
        this->gains = (pr_msgs::PIDgains*)realloc(this->gains, gains_lengthT * sizeof(pr_msgs::PIDgains));
      offset += 3;
      gains_length = gains_lengthT;
      for( uint8_t i = 0; i < gains_length; i++){
      offset += this->st_gains.deserialize(inbuffer + offset);
        memcpy( &(this->gains[i]), &(this->st_gains), sizeof(pr_msgs::PIDgains));
      }
     return offset;
    }

    const char * getType(){ return "pr_msgs/WAMInternals"; };
    const char * getMD5(){ return "1d5ca374a9297bbc0affedeafe993ccd"; };

  };

}
#endif