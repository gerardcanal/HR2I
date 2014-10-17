#ifndef _ROS_pr_msgs_WAMState_h
#define _ROS_pr_msgs_WAMState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "pr_msgs/TrajInfo.h"

namespace pr_msgs
{

  class WAMState : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t positions_length;
      double st_positions;
      double * positions;
      uint8_t jpositions_length;
      double st_jpositions;
      double * jpositions;
      uint8_t torques_length;
      double st_torques;
      double * torques;
      uint8_t trajectory_queue_length;
      pr_msgs::TrajInfo st_trajectory_queue;
      pr_msgs::TrajInfo * trajectory_queue;
      pr_msgs::TrajInfo prev_trajectory;
      uint8_t state;
      enum { state_free = 0 };
      enum { state_fixed = 1 };
      enum { state_traj_active = 2 };
      enum { state_traj_stalled = 3 };
      enum { state_traj_paused = 4 };
      enum { state_inactive = 255 };
      enum { state_moving = 2  };
      enum { state_stalled = 3  };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
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
      *(outbuffer + offset++) = jpositions_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < jpositions_length; i++){
      union {
        double real;
        uint64_t base;
      } u_jpositionsi;
      u_jpositionsi.real = this->jpositions[i];
      *(outbuffer + offset + 0) = (u_jpositionsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_jpositionsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_jpositionsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_jpositionsi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_jpositionsi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_jpositionsi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_jpositionsi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_jpositionsi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->jpositions[i]);
      }
      *(outbuffer + offset++) = torques_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < torques_length; i++){
      union {
        double real;
        uint64_t base;
      } u_torquesi;
      u_torquesi.real = this->torques[i];
      *(outbuffer + offset + 0) = (u_torquesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_torquesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_torquesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_torquesi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_torquesi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_torquesi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_torquesi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_torquesi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->torques[i]);
      }
      *(outbuffer + offset++) = trajectory_queue_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < trajectory_queue_length; i++){
      offset += this->trajectory_queue[i].serialize(outbuffer + offset);
      }
      offset += this->prev_trajectory.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
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
      uint8_t jpositions_lengthT = *(inbuffer + offset++);
      if(jpositions_lengthT > jpositions_length)
        this->jpositions = (double*)realloc(this->jpositions, jpositions_lengthT * sizeof(double));
      offset += 3;
      jpositions_length = jpositions_lengthT;
      for( uint8_t i = 0; i < jpositions_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_jpositions;
      u_st_jpositions.base = 0;
      u_st_jpositions.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_jpositions.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_jpositions.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_jpositions.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_jpositions.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_jpositions.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_jpositions.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_jpositions.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_jpositions = u_st_jpositions.real;
      offset += sizeof(this->st_jpositions);
        memcpy( &(this->jpositions[i]), &(this->st_jpositions), sizeof(double));
      }
      uint8_t torques_lengthT = *(inbuffer + offset++);
      if(torques_lengthT > torques_length)
        this->torques = (double*)realloc(this->torques, torques_lengthT * sizeof(double));
      offset += 3;
      torques_length = torques_lengthT;
      for( uint8_t i = 0; i < torques_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_torques;
      u_st_torques.base = 0;
      u_st_torques.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_torques.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_torques.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_torques.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_torques.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_torques.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_torques.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_torques.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_torques = u_st_torques.real;
      offset += sizeof(this->st_torques);
        memcpy( &(this->torques[i]), &(this->st_torques), sizeof(double));
      }
      uint8_t trajectory_queue_lengthT = *(inbuffer + offset++);
      if(trajectory_queue_lengthT > trajectory_queue_length)
        this->trajectory_queue = (pr_msgs::TrajInfo*)realloc(this->trajectory_queue, trajectory_queue_lengthT * sizeof(pr_msgs::TrajInfo));
      offset += 3;
      trajectory_queue_length = trajectory_queue_lengthT;
      for( uint8_t i = 0; i < trajectory_queue_length; i++){
      offset += this->st_trajectory_queue.deserialize(inbuffer + offset);
        memcpy( &(this->trajectory_queue[i]), &(this->st_trajectory_queue), sizeof(pr_msgs::TrajInfo));
      }
      offset += this->prev_trajectory.deserialize(inbuffer + offset);
      this->state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->state);
     return offset;
    }

    const char * getType(){ return "pr_msgs/WAMState"; };
    const char * getMD5(){ return "25cd353827aaf5484b1466979582c59d"; };

  };

}
#endif