#ifndef _ROS_SERVICE_SetJointStiffness_h
#define _ROS_SERVICE_SetJointStiffness_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

static const char SETJOINTSTIFFNESS[] = "pr_msgs/SetJointStiffness";

  class SetJointStiffnessRequest : public ros::Msg
  {
    public:
      uint8_t stiffness_length;
      float st_stiffness;
      float * stiffness;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = stiffness_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < stiffness_length; i++){
      union {
        float real;
        uint32_t base;
      } u_stiffnessi;
      u_stiffnessi.real = this->stiffness[i];
      *(outbuffer + offset + 0) = (u_stiffnessi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_stiffnessi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_stiffnessi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_stiffnessi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stiffness[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t stiffness_lengthT = *(inbuffer + offset++);
      if(stiffness_lengthT > stiffness_length)
        this->stiffness = (float*)realloc(this->stiffness, stiffness_lengthT * sizeof(float));
      offset += 3;
      stiffness_length = stiffness_lengthT;
      for( uint8_t i = 0; i < stiffness_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_stiffness;
      u_st_stiffness.base = 0;
      u_st_stiffness.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_stiffness.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_stiffness.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_stiffness.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_stiffness = u_st_stiffness.real;
      offset += sizeof(this->st_stiffness);
        memcpy( &(this->stiffness[i]), &(this->st_stiffness), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return SETJOINTSTIFFNESS; };
    const char * getMD5(){ return "3e07126e33eda8ee3f9b2b92a99f9528"; };

  };

  class SetJointStiffnessResponse : public ros::Msg
  {
    public:
      bool ok;
      char * reason;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ok;
      u_ok.real = this->ok;
      *(outbuffer + offset + 0) = (u_ok.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ok);
      uint32_t length_reason = strlen( (const char*) this->reason);
      memcpy(outbuffer + offset, &length_reason, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->reason, length_reason);
      offset += length_reason;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ok;
      u_ok.base = 0;
      u_ok.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ok = u_ok.real;
      offset += sizeof(this->ok);
      uint32_t length_reason;
      memcpy(&length_reason, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_reason; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_reason-1]=0;
      this->reason = (char *)(inbuffer + offset-1);
      offset += length_reason;
     return offset;
    }

    const char * getType(){ return SETJOINTSTIFFNESS; };
    const char * getMD5(){ return "4679398f882e7cbdea165980d3ec2888"; };

  };

  class SetJointStiffness {
    public:
    typedef SetJointStiffnessRequest Request;
    typedef SetJointStiffnessResponse Response;
  };

}
#endif
