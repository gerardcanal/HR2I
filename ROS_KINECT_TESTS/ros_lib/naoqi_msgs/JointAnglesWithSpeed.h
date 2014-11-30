#ifndef _ROS_naoqi_msgs_JointAnglesWithSpeed_h
#define _ROS_naoqi_msgs_JointAnglesWithSpeed_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace naoqi_msgs
{

  class JointAnglesWithSpeed : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t joint_names_length;
      char* st_joint_names;
      char* * joint_names;
      uint8_t joint_angles_length;
      float st_joint_angles;
      float * joint_angles;
      float speed;
      uint8_t relative;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = joint_names_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < joint_names_length; i++){
      uint32_t length_joint_namesi = strlen(this->joint_names[i]);
      memcpy(outbuffer + offset, &length_joint_namesi, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->joint_names[i], length_joint_namesi);
      offset += length_joint_namesi;
      }
      *(outbuffer + offset++) = joint_angles_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < joint_angles_length; i++){
      union {
        float real;
        uint32_t base;
      } u_joint_anglesi;
      u_joint_anglesi.real = this->joint_angles[i];
      *(outbuffer + offset + 0) = (u_joint_anglesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_anglesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_anglesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_anglesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_angles[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_speed;
      u_speed.real = this->speed;
      *(outbuffer + offset + 0) = (u_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed);
      *(outbuffer + offset + 0) = (this->relative >> (8 * 0)) & 0xFF;
      offset += sizeof(this->relative);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t joint_names_lengthT = *(inbuffer + offset++);
      if(joint_names_lengthT > joint_names_length)
        this->joint_names = (char**)realloc(this->joint_names, joint_names_lengthT * sizeof(char*));
      offset += 3;
      joint_names_length = joint_names_lengthT;
      for( uint8_t i = 0; i < joint_names_length; i++){
      uint32_t length_st_joint_names;
      memcpy(&length_st_joint_names, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_joint_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_joint_names-1]=0;
      this->st_joint_names = (char *)(inbuffer + offset-1);
      offset += length_st_joint_names;
        memcpy( &(this->joint_names[i]), &(this->st_joint_names), sizeof(char*));
      }
      uint8_t joint_angles_lengthT = *(inbuffer + offset++);
      if(joint_angles_lengthT > joint_angles_length)
        this->joint_angles = (float*)realloc(this->joint_angles, joint_angles_lengthT * sizeof(float));
      offset += 3;
      joint_angles_length = joint_angles_lengthT;
      for( uint8_t i = 0; i < joint_angles_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_joint_angles;
      u_st_joint_angles.base = 0;
      u_st_joint_angles.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_joint_angles.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_joint_angles.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_joint_angles.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_joint_angles = u_st_joint_angles.real;
      offset += sizeof(this->st_joint_angles);
        memcpy( &(this->joint_angles[i]), &(this->st_joint_angles), sizeof(float));
      }
      union {
        float real;
        uint32_t base;
      } u_speed;
      u_speed.base = 0;
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed = u_speed.real;
      offset += sizeof(this->speed);
      this->relative =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->relative);
     return offset;
    }

    const char * getType(){ return "naoqi_msgs/JointAnglesWithSpeed"; };
    const char * getMD5(){ return "052ca11f74a00ad6745dfff6ebc2b4d8"; };

  };

}
#endif