#ifndef _ROS_SERVICE_ReplaceTrajectory_h
#define _ROS_SERVICE_ReplaceTrajectory_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr_msgs/JointTraj.h"

namespace pr_msgs
{

static const char REPLACETRAJECTORY[] = "pr_msgs/ReplaceTrajectory";

  class ReplaceTrajectoryRequest : public ros::Msg
  {
    public:
      pr_msgs::JointTraj traj;
      uint32_t id;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->traj.serialize(outbuffer + offset);
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
      offset += this->traj.deserialize(inbuffer + offset);
      this->id =  ((uint32_t) (*(inbuffer + offset)));
      this->id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->id);
     return offset;
    }

    const char * getType(){ return REPLACETRAJECTORY; };
    const char * getMD5(){ return "f88b0684f828fc14e642b3386053416d"; };

  };

  class ReplaceTrajectoryResponse : public ros::Msg
  {
    public:
      uint32_t id;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
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
      this->id =  ((uint32_t) (*(inbuffer + offset)));
      this->id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->id);
     return offset;
    }

    const char * getType(){ return REPLACETRAJECTORY; };
    const char * getMD5(){ return "309d4b30834b338cced19e5622a97a03"; };

  };

  class ReplaceTrajectory {
    public:
    typedef ReplaceTrajectoryRequest Request;
    typedef ReplaceTrajectoryResponse Response;
  };

}
#endif
