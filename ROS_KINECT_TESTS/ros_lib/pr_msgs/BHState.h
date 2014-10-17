#ifndef _ROS_pr_msgs_BHState_h
#define _ROS_pr_msgs_BHState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace pr_msgs
{

  class BHState : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t positions_length;
      double st_positions;
      double * positions;
      uint8_t inner_links_length;
      double st_inner_links;
      double * inner_links;
      uint8_t outer_links_length;
      double st_outer_links;
      double * outer_links;
      uint8_t breakaway_length;
      bool st_breakaway;
      bool * breakaway;
      uint8_t strain_length;
      double st_strain;
      double * strain;
      uint8_t puck_temp_C_length;
      int32_t st_puck_temp_C;
      int32_t * puck_temp_C;
      uint8_t motor_temp_C_length;
      int32_t st_motor_temp_C;
      int32_t * motor_temp_C;
      uint8_t state;
      uint8_t internal_state_length;
      uint8_t st_internal_state;
      uint8_t * internal_state;
      uint8_t puck_mode_length;
      uint8_t st_puck_mode;
      uint8_t * puck_mode;
      enum { state_done = 1 };
      enum { state_moving = 2 };
      enum { state_stalled = 3 };
      enum { state_uninitialized = 255 };

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
      *(outbuffer + offset++) = inner_links_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < inner_links_length; i++){
      union {
        double real;
        uint64_t base;
      } u_inner_linksi;
      u_inner_linksi.real = this->inner_links[i];
      *(outbuffer + offset + 0) = (u_inner_linksi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_inner_linksi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_inner_linksi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_inner_linksi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_inner_linksi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_inner_linksi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_inner_linksi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_inner_linksi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->inner_links[i]);
      }
      *(outbuffer + offset++) = outer_links_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < outer_links_length; i++){
      union {
        double real;
        uint64_t base;
      } u_outer_linksi;
      u_outer_linksi.real = this->outer_links[i];
      *(outbuffer + offset + 0) = (u_outer_linksi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_outer_linksi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_outer_linksi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_outer_linksi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_outer_linksi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_outer_linksi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_outer_linksi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_outer_linksi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->outer_links[i]);
      }
      *(outbuffer + offset++) = breakaway_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < breakaway_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_breakawayi;
      u_breakawayi.real = this->breakaway[i];
      *(outbuffer + offset + 0) = (u_breakawayi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->breakaway[i]);
      }
      *(outbuffer + offset++) = strain_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < strain_length; i++){
      union {
        double real;
        uint64_t base;
      } u_straini;
      u_straini.real = this->strain[i];
      *(outbuffer + offset + 0) = (u_straini.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_straini.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_straini.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_straini.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_straini.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_straini.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_straini.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_straini.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->strain[i]);
      }
      *(outbuffer + offset++) = puck_temp_C_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < puck_temp_C_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_puck_temp_Ci;
      u_puck_temp_Ci.real = this->puck_temp_C[i];
      *(outbuffer + offset + 0) = (u_puck_temp_Ci.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_puck_temp_Ci.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_puck_temp_Ci.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_puck_temp_Ci.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->puck_temp_C[i]);
      }
      *(outbuffer + offset++) = motor_temp_C_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < motor_temp_C_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_motor_temp_Ci;
      u_motor_temp_Ci.real = this->motor_temp_C[i];
      *(outbuffer + offset + 0) = (u_motor_temp_Ci.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_temp_Ci.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_temp_Ci.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_temp_Ci.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_temp_C[i]);
      }
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      *(outbuffer + offset++) = internal_state_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < internal_state_length; i++){
      *(outbuffer + offset + 0) = (this->internal_state[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->internal_state[i]);
      }
      *(outbuffer + offset++) = puck_mode_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < puck_mode_length; i++){
      *(outbuffer + offset + 0) = (this->puck_mode[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->puck_mode[i]);
      }
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
      uint8_t inner_links_lengthT = *(inbuffer + offset++);
      if(inner_links_lengthT > inner_links_length)
        this->inner_links = (double*)realloc(this->inner_links, inner_links_lengthT * sizeof(double));
      offset += 3;
      inner_links_length = inner_links_lengthT;
      for( uint8_t i = 0; i < inner_links_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_inner_links;
      u_st_inner_links.base = 0;
      u_st_inner_links.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_inner_links.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_inner_links.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_inner_links.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_inner_links.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_inner_links.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_inner_links.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_inner_links.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_inner_links = u_st_inner_links.real;
      offset += sizeof(this->st_inner_links);
        memcpy( &(this->inner_links[i]), &(this->st_inner_links), sizeof(double));
      }
      uint8_t outer_links_lengthT = *(inbuffer + offset++);
      if(outer_links_lengthT > outer_links_length)
        this->outer_links = (double*)realloc(this->outer_links, outer_links_lengthT * sizeof(double));
      offset += 3;
      outer_links_length = outer_links_lengthT;
      for( uint8_t i = 0; i < outer_links_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_outer_links;
      u_st_outer_links.base = 0;
      u_st_outer_links.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_outer_links.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_outer_links.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_outer_links.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_outer_links.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_outer_links.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_outer_links.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_outer_links.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_outer_links = u_st_outer_links.real;
      offset += sizeof(this->st_outer_links);
        memcpy( &(this->outer_links[i]), &(this->st_outer_links), sizeof(double));
      }
      uint8_t breakaway_lengthT = *(inbuffer + offset++);
      if(breakaway_lengthT > breakaway_length)
        this->breakaway = (bool*)realloc(this->breakaway, breakaway_lengthT * sizeof(bool));
      offset += 3;
      breakaway_length = breakaway_lengthT;
      for( uint8_t i = 0; i < breakaway_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_st_breakaway;
      u_st_breakaway.base = 0;
      u_st_breakaway.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_breakaway = u_st_breakaway.real;
      offset += sizeof(this->st_breakaway);
        memcpy( &(this->breakaway[i]), &(this->st_breakaway), sizeof(bool));
      }
      uint8_t strain_lengthT = *(inbuffer + offset++);
      if(strain_lengthT > strain_length)
        this->strain = (double*)realloc(this->strain, strain_lengthT * sizeof(double));
      offset += 3;
      strain_length = strain_lengthT;
      for( uint8_t i = 0; i < strain_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_strain;
      u_st_strain.base = 0;
      u_st_strain.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_strain.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_strain.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_strain.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_strain.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_strain.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_strain.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_strain.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_strain = u_st_strain.real;
      offset += sizeof(this->st_strain);
        memcpy( &(this->strain[i]), &(this->st_strain), sizeof(double));
      }
      uint8_t puck_temp_C_lengthT = *(inbuffer + offset++);
      if(puck_temp_C_lengthT > puck_temp_C_length)
        this->puck_temp_C = (int32_t*)realloc(this->puck_temp_C, puck_temp_C_lengthT * sizeof(int32_t));
      offset += 3;
      puck_temp_C_length = puck_temp_C_lengthT;
      for( uint8_t i = 0; i < puck_temp_C_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_puck_temp_C;
      u_st_puck_temp_C.base = 0;
      u_st_puck_temp_C.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_puck_temp_C.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_puck_temp_C.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_puck_temp_C.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_puck_temp_C = u_st_puck_temp_C.real;
      offset += sizeof(this->st_puck_temp_C);
        memcpy( &(this->puck_temp_C[i]), &(this->st_puck_temp_C), sizeof(int32_t));
      }
      uint8_t motor_temp_C_lengthT = *(inbuffer + offset++);
      if(motor_temp_C_lengthT > motor_temp_C_length)
        this->motor_temp_C = (int32_t*)realloc(this->motor_temp_C, motor_temp_C_lengthT * sizeof(int32_t));
      offset += 3;
      motor_temp_C_length = motor_temp_C_lengthT;
      for( uint8_t i = 0; i < motor_temp_C_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_motor_temp_C;
      u_st_motor_temp_C.base = 0;
      u_st_motor_temp_C.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_motor_temp_C.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_motor_temp_C.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_motor_temp_C.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_motor_temp_C = u_st_motor_temp_C.real;
      offset += sizeof(this->st_motor_temp_C);
        memcpy( &(this->motor_temp_C[i]), &(this->st_motor_temp_C), sizeof(int32_t));
      }
      this->state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->state);
      uint8_t internal_state_lengthT = *(inbuffer + offset++);
      if(internal_state_lengthT > internal_state_length)
        this->internal_state = (uint8_t*)realloc(this->internal_state, internal_state_lengthT * sizeof(uint8_t));
      offset += 3;
      internal_state_length = internal_state_lengthT;
      for( uint8_t i = 0; i < internal_state_length; i++){
      this->st_internal_state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_internal_state);
        memcpy( &(this->internal_state[i]), &(this->st_internal_state), sizeof(uint8_t));
      }
      uint8_t puck_mode_lengthT = *(inbuffer + offset++);
      if(puck_mode_lengthT > puck_mode_length)
        this->puck_mode = (uint8_t*)realloc(this->puck_mode, puck_mode_lengthT * sizeof(uint8_t));
      offset += 3;
      puck_mode_length = puck_mode_lengthT;
      for( uint8_t i = 0; i < puck_mode_length; i++){
      this->st_puck_mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_puck_mode);
        memcpy( &(this->puck_mode[i]), &(this->st_puck_mode), sizeof(uint8_t));
      }
     return offset;
    }

    const char * getType(){ return "pr_msgs/BHState"; };
    const char * getMD5(){ return "e6b6dfb306cd8c934654e8dd288b25fb"; };

  };

}
#endif