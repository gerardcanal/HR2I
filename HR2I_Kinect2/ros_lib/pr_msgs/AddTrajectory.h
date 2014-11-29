#ifndef _ROS_SERVICE_AddTrajectory_h
#define _ROS_SERVICE_AddTrajectory_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr_msgs/JointTraj.h"

namespace pr_msgs
{

static const char ADDTRAJECTORY[] = "pr_msgs/AddTrajectory";

  class AddTrajectoryRequest : public ros::Msg
  {
    public:
      pr_msgs::JointTraj traj;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->traj.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->traj.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return ADDTRAJECTORY; };
    const char * getMD5(){ return "dac72239f9f69a87ec7747c7a90f2ad9"; };

  };

  class AddTrajectoryResponse : public ros::Msg
  {
    public:
      bool ok;
      char * reason;
      uint32_t id;

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
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
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
      this->id =  ((uint32_t) (*(inbuffer + offset)));
      this->id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->id);
     return offset;
    }

    const char * getType(){ return ADDTRAJECTORY; };
    const char * getMD5(){ return "ad2e2c70d0557970f2c1628f5539a6dd"; };

  };

  class AddTrajectory {
    public:
    typedef AddTrajectoryRequest Request;
    typedef AddTrajectoryResponse Response;
  };

}
#endif
