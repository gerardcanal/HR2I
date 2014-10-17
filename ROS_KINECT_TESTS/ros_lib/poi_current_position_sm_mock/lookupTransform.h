#ifndef _ROS_SERVICE_lookupTransform_h
#define _ROS_SERVICE_lookupTransform_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/time.h"

namespace poi_current_position_sm_mock
{

static const char LOOKUPTRANSFORM[] = "poi_current_position_sm_mock/lookupTransform";

  class lookupTransformRequest : public ros::Msg
  {
    public:
      char * target_frame;
      char * source_frame;
      ros::Time transform_time;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_target_frame = strlen( (const char*) this->target_frame);
      memcpy(outbuffer + offset, &length_target_frame, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->target_frame, length_target_frame);
      offset += length_target_frame;
      uint32_t length_source_frame = strlen( (const char*) this->source_frame);
      memcpy(outbuffer + offset, &length_source_frame, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->source_frame, length_source_frame);
      offset += length_source_frame;
      *(outbuffer + offset + 0) = (this->transform_time.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->transform_time.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->transform_time.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->transform_time.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->transform_time.sec);
      *(outbuffer + offset + 0) = (this->transform_time.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->transform_time.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->transform_time.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->transform_time.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->transform_time.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_target_frame;
      memcpy(&length_target_frame, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_target_frame; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_target_frame-1]=0;
      this->target_frame = (char *)(inbuffer + offset-1);
      offset += length_target_frame;
      uint32_t length_source_frame;
      memcpy(&length_source_frame, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_source_frame; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_source_frame-1]=0;
      this->source_frame = (char *)(inbuffer + offset-1);
      offset += length_source_frame;
      this->transform_time.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->transform_time.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->transform_time.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->transform_time.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->transform_time.sec);
      this->transform_time.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->transform_time.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->transform_time.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->transform_time.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->transform_time.nsec);
     return offset;
    }

    const char * getType(){ return LOOKUPTRANSFORM; };
    const char * getMD5(){ return "bb9d983758e61f286b43546ac9c0b080"; };

  };

  class lookupTransformResponse : public ros::Msg
  {
    public:
      geometry_msgs::TransformStamped transform;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->transform.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->transform.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return LOOKUPTRANSFORM; };
    const char * getMD5(){ return "627ebb4e09bbb127f87308bbfdbaec08"; };

  };

  class lookupTransform {
    public:
    typedef lookupTransformRequest Request;
    typedef lookupTransformResponse Response;
  };

}
#endif
