#ifndef _ROS_SERVICE_SetTactileInputThreshold_h
#define _ROS_SERVICE_SetTactileInputThreshold_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

static const char SETTACTILEINPUTTHRESHOLD[] = "pr_msgs/SetTactileInputThreshold";

  class SetTactileInputThresholdRequest : public ros::Msg
  {
    public:
      int32_t pad_number;
      float threshold;
      int32_t minimum_readings;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_pad_number;
      u_pad_number.real = this->pad_number;
      *(outbuffer + offset + 0) = (u_pad_number.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pad_number.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pad_number.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pad_number.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pad_number);
      union {
        float real;
        uint32_t base;
      } u_threshold;
      u_threshold.real = this->threshold;
      *(outbuffer + offset + 0) = (u_threshold.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_threshold.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_threshold.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_threshold.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->threshold);
      union {
        int32_t real;
        uint32_t base;
      } u_minimum_readings;
      u_minimum_readings.real = this->minimum_readings;
      *(outbuffer + offset + 0) = (u_minimum_readings.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_minimum_readings.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_minimum_readings.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_minimum_readings.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->minimum_readings);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_pad_number;
      u_pad_number.base = 0;
      u_pad_number.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pad_number.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pad_number.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pad_number.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pad_number = u_pad_number.real;
      offset += sizeof(this->pad_number);
      union {
        float real;
        uint32_t base;
      } u_threshold;
      u_threshold.base = 0;
      u_threshold.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_threshold.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_threshold.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_threshold.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->threshold = u_threshold.real;
      offset += sizeof(this->threshold);
      union {
        int32_t real;
        uint32_t base;
      } u_minimum_readings;
      u_minimum_readings.base = 0;
      u_minimum_readings.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_minimum_readings.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_minimum_readings.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_minimum_readings.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->minimum_readings = u_minimum_readings.real;
      offset += sizeof(this->minimum_readings);
     return offset;
    }

    const char * getType(){ return SETTACTILEINPUTTHRESHOLD; };
    const char * getMD5(){ return "12ef4e765b52e2818bb6194890021669"; };

  };

  class SetTactileInputThresholdResponse : public ros::Msg
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

    const char * getType(){ return SETTACTILEINPUTTHRESHOLD; };
    const char * getMD5(){ return "4679398f882e7cbdea165980d3ec2888"; };

  };

  class SetTactileInputThreshold {
    public:
    typedef SetTactileInputThresholdRequest Request;
    typedef SetTactileInputThresholdResponse Response;
  };

}
#endif
