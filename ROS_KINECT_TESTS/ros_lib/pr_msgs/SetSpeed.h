#ifndef _ROS_SERVICE_SetSpeed_h
#define _ROS_SERVICE_SetSpeed_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

static const char SETSPEED[] = "pr_msgs/SetSpeed";

  class SetSpeedRequest : public ros::Msg
  {
    public:
      uint8_t velocities_length;
      double st_velocities;
      double * velocities;
      double min_accel_time;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = velocities_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < velocities_length; i++){
      union {
        double real;
        uint64_t base;
      } u_velocitiesi;
      u_velocitiesi.real = this->velocities[i];
      *(outbuffer + offset + 0) = (u_velocitiesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocitiesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocitiesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocitiesi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_velocitiesi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_velocitiesi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_velocitiesi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_velocitiesi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->velocities[i]);
      }
      union {
        double real;
        uint64_t base;
      } u_min_accel_time;
      u_min_accel_time.real = this->min_accel_time;
      *(outbuffer + offset + 0) = (u_min_accel_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_accel_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_accel_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_accel_time.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_min_accel_time.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_min_accel_time.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_min_accel_time.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_min_accel_time.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->min_accel_time);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t velocities_lengthT = *(inbuffer + offset++);
      if(velocities_lengthT > velocities_length)
        this->velocities = (double*)realloc(this->velocities, velocities_lengthT * sizeof(double));
      offset += 3;
      velocities_length = velocities_lengthT;
      for( uint8_t i = 0; i < velocities_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_velocities;
      u_st_velocities.base = 0;
      u_st_velocities.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_velocities.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_velocities.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_velocities.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_velocities.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_velocities.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_velocities.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_velocities.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_velocities = u_st_velocities.real;
      offset += sizeof(this->st_velocities);
        memcpy( &(this->velocities[i]), &(this->st_velocities), sizeof(double));
      }
      union {
        double real;
        uint64_t base;
      } u_min_accel_time;
      u_min_accel_time.base = 0;
      u_min_accel_time.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_accel_time.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_accel_time.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_accel_time.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_min_accel_time.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_min_accel_time.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_min_accel_time.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_min_accel_time.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->min_accel_time = u_min_accel_time.real;
      offset += sizeof(this->min_accel_time);
     return offset;
    }

    const char * getType(){ return SETSPEED; };
    const char * getMD5(){ return "f25539e8e9259120cd8f44c968d7cd7a"; };

  };

  class SetSpeedResponse : public ros::Msg
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

    const char * getType(){ return SETSPEED; };
    const char * getMD5(){ return "4679398f882e7cbdea165980d3ec2888"; };

  };

  class SetSpeed {
    public:
    typedef SetSpeedRequest Request;
    typedef SetSpeedResponse Response;
  };

}
#endif
