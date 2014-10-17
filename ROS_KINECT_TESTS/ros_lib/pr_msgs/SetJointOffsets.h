#ifndef _ROS_SERVICE_SetJointOffsets_h
#define _ROS_SERVICE_SetJointOffsets_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

static const char SETJOINTOFFSETS[] = "pr_msgs/SetJointOffsets";

  class SetJointOffsetsRequest : public ros::Msg
  {
    public:
      uint8_t offset_length;
      double st_offset;
      double * offset;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = offset_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < offset_length; i++){
      union {
        double real;
        uint64_t base;
      } u_offseti;
      u_offseti.real = this->offset[i];
      *(outbuffer + offset + 0) = (u_offseti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_offseti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_offseti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_offseti.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_offseti.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_offseti.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_offseti.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_offseti.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->offset[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t offset_lengthT = *(inbuffer + offset++);
      if(offset_lengthT > offset_length)
        this->offset = (double*)realloc(this->offset, offset_lengthT * sizeof(double));
      offset += 3;
      offset_length = offset_lengthT;
      for( uint8_t i = 0; i < offset_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_offset;
      u_st_offset.base = 0;
      u_st_offset.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_offset.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_offset.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_offset.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_offset.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_offset.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_offset.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_offset.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_offset = u_st_offset.real;
      offset += sizeof(this->st_offset);
        memcpy( &(this->offset[i]), &(this->st_offset), sizeof(double));
      }
     return offset;
    }

    const char * getType(){ return SETJOINTOFFSETS; };
    const char * getMD5(){ return "64d8bbc9ac6a377e0857cd689c98857c"; };

  };

  class SetJointOffsetsResponse : public ros::Msg
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

    const char * getType(){ return SETJOINTOFFSETS; };
    const char * getMD5(){ return "4679398f882e7cbdea165980d3ec2888"; };

  };

  class SetJointOffsets {
    public:
    typedef SetJointOffsetsRequest Request;
    typedef SetJointOffsetsResponse Response;
  };

}
#endif
