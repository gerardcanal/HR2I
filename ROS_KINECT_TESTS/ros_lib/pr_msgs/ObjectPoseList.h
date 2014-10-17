#ifndef _ROS_pr_msgs_ObjectPoseList_h
#define _ROS_pr_msgs_ObjectPoseList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "pr_msgs/ObjectPose.h"
#include "ros/time.h"

namespace pr_msgs
{

  class ObjectPoseList : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t object_list_length;
      pr_msgs::ObjectPose st_object_list;
      pr_msgs::ObjectPose * object_list;
      ros::Time originalTimeStamp;
      ros::Time requestTimeStamp;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = object_list_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < object_list_length; i++){
      offset += this->object_list[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->originalTimeStamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->originalTimeStamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->originalTimeStamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->originalTimeStamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->originalTimeStamp.sec);
      *(outbuffer + offset + 0) = (this->originalTimeStamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->originalTimeStamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->originalTimeStamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->originalTimeStamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->originalTimeStamp.nsec);
      *(outbuffer + offset + 0) = (this->requestTimeStamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->requestTimeStamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->requestTimeStamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->requestTimeStamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->requestTimeStamp.sec);
      *(outbuffer + offset + 0) = (this->requestTimeStamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->requestTimeStamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->requestTimeStamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->requestTimeStamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->requestTimeStamp.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t object_list_lengthT = *(inbuffer + offset++);
      if(object_list_lengthT > object_list_length)
        this->object_list = (pr_msgs::ObjectPose*)realloc(this->object_list, object_list_lengthT * sizeof(pr_msgs::ObjectPose));
      offset += 3;
      object_list_length = object_list_lengthT;
      for( uint8_t i = 0; i < object_list_length; i++){
      offset += this->st_object_list.deserialize(inbuffer + offset);
        memcpy( &(this->object_list[i]), &(this->st_object_list), sizeof(pr_msgs::ObjectPose));
      }
      this->originalTimeStamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->originalTimeStamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->originalTimeStamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->originalTimeStamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->originalTimeStamp.sec);
      this->originalTimeStamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->originalTimeStamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->originalTimeStamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->originalTimeStamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->originalTimeStamp.nsec);
      this->requestTimeStamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->requestTimeStamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->requestTimeStamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->requestTimeStamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->requestTimeStamp.sec);
      this->requestTimeStamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->requestTimeStamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->requestTimeStamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->requestTimeStamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->requestTimeStamp.nsec);
     return offset;
    }

    const char * getType(){ return "pr_msgs/ObjectPoseList"; };
    const char * getMD5(){ return "418ab919a68374930e8c523eaea185d2"; };

  };

}
#endif