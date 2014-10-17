#ifndef _ROS_pr_msgs_TrajInfo_h
#define _ROS_pr_msgs_TrajInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

  class TrajInfo : public ros::Msg
  {
    public:
      uint32_t id;
      char * type;
      uint8_t end_position_length;
      double st_end_position;
      double * end_position;
      uint32_t state;
      enum { state_pending = 0 };
      enum { state_active = 1 };
      enum { state_done = 3 };
      enum { state_aborted = 4 };
      enum { state_running = 1  };
      enum { state_paused = 2   };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      uint32_t length_type = strlen( (const char*) this->type);
      memcpy(outbuffer + offset, &length_type, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->type, length_type);
      offset += length_type;
      *(outbuffer + offset++) = end_position_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < end_position_length; i++){
      union {
        double real;
        uint64_t base;
      } u_end_positioni;
      u_end_positioni.real = this->end_position[i];
      *(outbuffer + offset + 0) = (u_end_positioni.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_end_positioni.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_end_positioni.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_end_positioni.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_end_positioni.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_end_positioni.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_end_positioni.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_end_positioni.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->end_position[i]);
      }
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->state >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->state >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->state >> (8 * 3)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->id =  ((uint32_t) (*(inbuffer + offset)));
      this->id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->id);
      uint32_t length_type;
      memcpy(&length_type, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_type-1]=0;
      this->type = (char *)(inbuffer + offset-1);
      offset += length_type;
      uint8_t end_position_lengthT = *(inbuffer + offset++);
      if(end_position_lengthT > end_position_length)
        this->end_position = (double*)realloc(this->end_position, end_position_lengthT * sizeof(double));
      offset += 3;
      end_position_length = end_position_lengthT;
      for( uint8_t i = 0; i < end_position_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_end_position;
      u_st_end_position.base = 0;
      u_st_end_position.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_end_position.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_end_position.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_end_position.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_end_position.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_end_position.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_end_position.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_end_position.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_end_position = u_st_end_position.real;
      offset += sizeof(this->st_end_position);
        memcpy( &(this->end_position[i]), &(this->st_end_position), sizeof(double));
      }
      this->state =  ((uint32_t) (*(inbuffer + offset)));
      this->state |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->state |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->state |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->state);
     return offset;
    }

    const char * getType(){ return "pr_msgs/TrajInfo"; };
    const char * getMD5(){ return "82f4530e5ffe77991769af4f85589db5"; };

  };

}
#endif