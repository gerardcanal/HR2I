#ifndef _ROS_pr_msgs_DefineLocation_h
#define _ROS_pr_msgs_DefineLocation_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace pr_msgs
{

  class DefineLocation : public ros::Msg
  {
    public:
      std_msgs::Header header;
      char * name;
      float xx;
      float yy;
      float th;
      float dist_thresh;
      float angle_thresh;
      uint8_t type;
      enum { type_place = 1	 };
      enum { type_thing = 2	 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_name = strlen( (const char*) this->name);
      memcpy(outbuffer + offset, &length_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      union {
        float real;
        uint32_t base;
      } u_xx;
      u_xx.real = this->xx;
      *(outbuffer + offset + 0) = (u_xx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->xx);
      union {
        float real;
        uint32_t base;
      } u_yy;
      u_yy.real = this->yy;
      *(outbuffer + offset + 0) = (u_yy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yy);
      union {
        float real;
        uint32_t base;
      } u_th;
      u_th.real = this->th;
      *(outbuffer + offset + 0) = (u_th.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_th.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_th.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_th.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->th);
      union {
        float real;
        uint32_t base;
      } u_dist_thresh;
      u_dist_thresh.real = this->dist_thresh;
      *(outbuffer + offset + 0) = (u_dist_thresh.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dist_thresh.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dist_thresh.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dist_thresh.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dist_thresh);
      union {
        float real;
        uint32_t base;
      } u_angle_thresh;
      u_angle_thresh.real = this->angle_thresh;
      *(outbuffer + offset + 0) = (u_angle_thresh.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle_thresh.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle_thresh.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle_thresh.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle_thresh);
      *(outbuffer + offset + 0) = (this->type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_name;
      memcpy(&length_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      union {
        float real;
        uint32_t base;
      } u_xx;
      u_xx.base = 0;
      u_xx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->xx = u_xx.real;
      offset += sizeof(this->xx);
      union {
        float real;
        uint32_t base;
      } u_yy;
      u_yy.base = 0;
      u_yy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yy = u_yy.real;
      offset += sizeof(this->yy);
      union {
        float real;
        uint32_t base;
      } u_th;
      u_th.base = 0;
      u_th.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_th.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_th.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_th.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->th = u_th.real;
      offset += sizeof(this->th);
      union {
        float real;
        uint32_t base;
      } u_dist_thresh;
      u_dist_thresh.base = 0;
      u_dist_thresh.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dist_thresh.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dist_thresh.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dist_thresh.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dist_thresh = u_dist_thresh.real;
      offset += sizeof(this->dist_thresh);
      union {
        float real;
        uint32_t base;
      } u_angle_thresh;
      u_angle_thresh.base = 0;
      u_angle_thresh.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle_thresh.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle_thresh.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle_thresh.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle_thresh = u_angle_thresh.real;
      offset += sizeof(this->angle_thresh);
      this->type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->type);
     return offset;
    }

    const char * getType(){ return "pr_msgs/DefineLocation"; };
    const char * getMD5(){ return "0be26fc27c554ba0d2e3b0713e9ab224"; };

  };

}
#endif