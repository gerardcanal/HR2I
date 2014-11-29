#ifndef _ROS_pr_msgs_WAMJointState_h
#define _ROS_pr_msgs_WAMJointState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace pr_msgs
{

  class WAMJointState : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t positions_length;
      double st_positions;
      double * positions;
      uint8_t positions_commanded_length;
      double st_positions_commanded;
      double * positions_commanded;
      uint8_t torques_length;
      double st_torques;
      double * torques;

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
      *(outbuffer + offset++) = positions_commanded_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < positions_commanded_length; i++){
      union {
        double real;
        uint64_t base;
      } u_positions_commandedi;
      u_positions_commandedi.real = this->positions_commanded[i];
      *(outbuffer + offset + 0) = (u_positions_commandedi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_positions_commandedi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_positions_commandedi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_positions_commandedi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_positions_commandedi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_positions_commandedi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_positions_commandedi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_positions_commandedi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->positions_commanded[i]);
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
      uint8_t positions_commanded_lengthT = *(inbuffer + offset++);
      if(positions_commanded_lengthT > positions_commanded_length)
        this->positions_commanded = (double*)realloc(this->positions_commanded, positions_commanded_lengthT * sizeof(double));
      offset += 3;
      positions_commanded_length = positions_commanded_lengthT;
      for( uint8_t i = 0; i < positions_commanded_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_positions_commanded;
      u_st_positions_commanded.base = 0;
      u_st_positions_commanded.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_positions_commanded.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_positions_commanded.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_positions_commanded.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_positions_commanded.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_positions_commanded.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_positions_commanded.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_positions_commanded.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_positions_commanded = u_st_positions_commanded.real;
      offset += sizeof(this->st_positions_commanded);
        memcpy( &(this->positions_commanded[i]), &(this->st_positions_commanded), sizeof(double));
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
     return offset;
    }

    const char * getType(){ return "pr_msgs/WAMJointState"; };
    const char * getMD5(){ return "9c5a0c31ef05f2ab6b4baacc5cbc9e0d"; };

  };

}
#endif