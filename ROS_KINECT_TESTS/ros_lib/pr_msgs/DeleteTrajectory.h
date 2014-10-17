#ifndef _ROS_SERVICE_DeleteTrajectory_h
#define _ROS_SERVICE_DeleteTrajectory_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

static const char DELETETRAJECTORY[] = "pr_msgs/DeleteTrajectory";

  class DeleteTrajectoryRequest : public ros::Msg
  {
    public:
      uint8_t ids_length;
      uint32_t st_ids;
      uint32_t * ids;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = ids_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < ids_length; i++){
      *(outbuffer + offset + 0) = (this->ids[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ids[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ids[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ids[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ids[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t ids_lengthT = *(inbuffer + offset++);
      if(ids_lengthT > ids_length)
        this->ids = (uint32_t*)realloc(this->ids, ids_lengthT * sizeof(uint32_t));
      offset += 3;
      ids_length = ids_lengthT;
      for( uint8_t i = 0; i < ids_length; i++){
      this->st_ids =  ((uint32_t) (*(inbuffer + offset)));
      this->st_ids |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_ids |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_ids |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_ids);
        memcpy( &(this->ids[i]), &(this->st_ids), sizeof(uint32_t));
      }
     return offset;
    }

    const char * getType(){ return DELETETRAJECTORY; };
    const char * getMD5(){ return "8e09436fae0f3b412c2e4947052cd55b"; };

  };

  class DeleteTrajectoryResponse : public ros::Msg
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

    const char * getType(){ return DELETETRAJECTORY; };
    const char * getMD5(){ return "4679398f882e7cbdea165980d3ec2888"; };

  };

  class DeleteTrajectory {
    public:
    typedef DeleteTrajectoryRequest Request;
    typedef DeleteTrajectoryResponse Response;
  };

}
#endif
