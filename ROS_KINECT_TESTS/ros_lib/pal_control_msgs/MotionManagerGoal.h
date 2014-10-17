#ifndef _ROS_pal_control_msgs_MotionManagerGoal_h
#define _ROS_pal_control_msgs_MotionManagerGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pal_control_msgs
{

  class MotionManagerGoal : public ros::Msg
  {
    public:
      char * filename;
      bool plan;
      bool checkSafety;
      bool repeat;
      uint8_t priority;
      int32_t queueTimeout;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_filename = strlen( (const char*) this->filename);
      memcpy(outbuffer + offset, &length_filename, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->filename, length_filename);
      offset += length_filename;
      union {
        bool real;
        uint8_t base;
      } u_plan;
      u_plan.real = this->plan;
      *(outbuffer + offset + 0) = (u_plan.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->plan);
      union {
        bool real;
        uint8_t base;
      } u_checkSafety;
      u_checkSafety.real = this->checkSafety;
      *(outbuffer + offset + 0) = (u_checkSafety.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->checkSafety);
      union {
        bool real;
        uint8_t base;
      } u_repeat;
      u_repeat.real = this->repeat;
      *(outbuffer + offset + 0) = (u_repeat.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->repeat);
      *(outbuffer + offset + 0) = (this->priority >> (8 * 0)) & 0xFF;
      offset += sizeof(this->priority);
      union {
        int32_t real;
        uint32_t base;
      } u_queueTimeout;
      u_queueTimeout.real = this->queueTimeout;
      *(outbuffer + offset + 0) = (u_queueTimeout.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_queueTimeout.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_queueTimeout.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_queueTimeout.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->queueTimeout);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_filename;
      memcpy(&length_filename, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_filename; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_filename-1]=0;
      this->filename = (char *)(inbuffer + offset-1);
      offset += length_filename;
      union {
        bool real;
        uint8_t base;
      } u_plan;
      u_plan.base = 0;
      u_plan.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->plan = u_plan.real;
      offset += sizeof(this->plan);
      union {
        bool real;
        uint8_t base;
      } u_checkSafety;
      u_checkSafety.base = 0;
      u_checkSafety.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->checkSafety = u_checkSafety.real;
      offset += sizeof(this->checkSafety);
      union {
        bool real;
        uint8_t base;
      } u_repeat;
      u_repeat.base = 0;
      u_repeat.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->repeat = u_repeat.real;
      offset += sizeof(this->repeat);
      this->priority =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->priority);
      union {
        int32_t real;
        uint32_t base;
      } u_queueTimeout;
      u_queueTimeout.base = 0;
      u_queueTimeout.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_queueTimeout.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_queueTimeout.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_queueTimeout.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->queueTimeout = u_queueTimeout.real;
      offset += sizeof(this->queueTimeout);
     return offset;
    }

    const char * getType(){ return "pal_control_msgs/MotionManagerGoal"; };
    const char * getMD5(){ return "6cfa0989c229a7ae793c273afdc78ead"; };

  };

}
#endif