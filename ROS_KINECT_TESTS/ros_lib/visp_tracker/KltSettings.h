#ifndef _ROS_visp_tracker_KltSettings_h
#define _ROS_visp_tracker_KltSettings_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace visp_tracker
{

  class KltSettings : public ros::Msg
  {
    public:
      int64_t max_features;
      int64_t window_size;
      double quality;
      double min_distance;
      double harris;
      int64_t size_block;
      int64_t pyramid_lvl;
      double angle_appear;
      double angle_disappear;
      int64_t mask_border;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_max_features;
      u_max_features.real = this->max_features;
      *(outbuffer + offset + 0) = (u_max_features.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_features.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_features.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_features.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_features.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_features.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_features.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_features.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_features);
      union {
        int64_t real;
        uint64_t base;
      } u_window_size;
      u_window_size.real = this->window_size;
      *(outbuffer + offset + 0) = (u_window_size.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_window_size.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_window_size.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_window_size.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_window_size.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_window_size.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_window_size.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_window_size.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->window_size);
      union {
        double real;
        uint64_t base;
      } u_quality;
      u_quality.real = this->quality;
      *(outbuffer + offset + 0) = (u_quality.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_quality.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_quality.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_quality.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_quality.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_quality.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_quality.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_quality.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->quality);
      union {
        double real;
        uint64_t base;
      } u_min_distance;
      u_min_distance.real = this->min_distance;
      *(outbuffer + offset + 0) = (u_min_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_distance.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_min_distance.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_min_distance.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_min_distance.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_min_distance.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->min_distance);
      union {
        double real;
        uint64_t base;
      } u_harris;
      u_harris.real = this->harris;
      *(outbuffer + offset + 0) = (u_harris.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_harris.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_harris.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_harris.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_harris.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_harris.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_harris.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_harris.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->harris);
      union {
        int64_t real;
        uint64_t base;
      } u_size_block;
      u_size_block.real = this->size_block;
      *(outbuffer + offset + 0) = (u_size_block.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_size_block.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_size_block.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_size_block.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_size_block.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_size_block.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_size_block.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_size_block.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->size_block);
      union {
        int64_t real;
        uint64_t base;
      } u_pyramid_lvl;
      u_pyramid_lvl.real = this->pyramid_lvl;
      *(outbuffer + offset + 0) = (u_pyramid_lvl.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pyramid_lvl.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pyramid_lvl.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pyramid_lvl.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_pyramid_lvl.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_pyramid_lvl.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_pyramid_lvl.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_pyramid_lvl.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->pyramid_lvl);
      union {
        double real;
        uint64_t base;
      } u_angle_appear;
      u_angle_appear.real = this->angle_appear;
      *(outbuffer + offset + 0) = (u_angle_appear.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle_appear.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle_appear.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle_appear.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_angle_appear.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_angle_appear.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_angle_appear.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_angle_appear.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->angle_appear);
      union {
        double real;
        uint64_t base;
      } u_angle_disappear;
      u_angle_disappear.real = this->angle_disappear;
      *(outbuffer + offset + 0) = (u_angle_disappear.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle_disappear.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle_disappear.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle_disappear.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_angle_disappear.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_angle_disappear.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_angle_disappear.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_angle_disappear.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->angle_disappear);
      union {
        int64_t real;
        uint64_t base;
      } u_mask_border;
      u_mask_border.real = this->mask_border;
      *(outbuffer + offset + 0) = (u_mask_border.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mask_border.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mask_border.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mask_border.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mask_border.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mask_border.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mask_border.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mask_border.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mask_border);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_max_features;
      u_max_features.base = 0;
      u_max_features.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_features.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_features.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_features.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_max_features.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_max_features.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_max_features.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_max_features.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->max_features = u_max_features.real;
      offset += sizeof(this->max_features);
      union {
        int64_t real;
        uint64_t base;
      } u_window_size;
      u_window_size.base = 0;
      u_window_size.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_window_size.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_window_size.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_window_size.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_window_size.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_window_size.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_window_size.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_window_size.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->window_size = u_window_size.real;
      offset += sizeof(this->window_size);
      union {
        double real;
        uint64_t base;
      } u_quality;
      u_quality.base = 0;
      u_quality.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_quality.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_quality.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_quality.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_quality.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_quality.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_quality.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_quality.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->quality = u_quality.real;
      offset += sizeof(this->quality);
      union {
        double real;
        uint64_t base;
      } u_min_distance;
      u_min_distance.base = 0;
      u_min_distance.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_distance.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_distance.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_distance.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_min_distance.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_min_distance.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_min_distance.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_min_distance.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->min_distance = u_min_distance.real;
      offset += sizeof(this->min_distance);
      union {
        double real;
        uint64_t base;
      } u_harris;
      u_harris.base = 0;
      u_harris.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_harris.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_harris.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_harris.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_harris.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_harris.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_harris.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_harris.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->harris = u_harris.real;
      offset += sizeof(this->harris);
      union {
        int64_t real;
        uint64_t base;
      } u_size_block;
      u_size_block.base = 0;
      u_size_block.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_size_block.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_size_block.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_size_block.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_size_block.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_size_block.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_size_block.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_size_block.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->size_block = u_size_block.real;
      offset += sizeof(this->size_block);
      union {
        int64_t real;
        uint64_t base;
      } u_pyramid_lvl;
      u_pyramid_lvl.base = 0;
      u_pyramid_lvl.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pyramid_lvl.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pyramid_lvl.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pyramid_lvl.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_pyramid_lvl.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_pyramid_lvl.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_pyramid_lvl.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_pyramid_lvl.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->pyramid_lvl = u_pyramid_lvl.real;
      offset += sizeof(this->pyramid_lvl);
      union {
        double real;
        uint64_t base;
      } u_angle_appear;
      u_angle_appear.base = 0;
      u_angle_appear.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle_appear.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle_appear.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle_appear.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_angle_appear.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_angle_appear.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_angle_appear.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_angle_appear.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->angle_appear = u_angle_appear.real;
      offset += sizeof(this->angle_appear);
      union {
        double real;
        uint64_t base;
      } u_angle_disappear;
      u_angle_disappear.base = 0;
      u_angle_disappear.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle_disappear.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle_disappear.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle_disappear.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_angle_disappear.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_angle_disappear.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_angle_disappear.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_angle_disappear.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->angle_disappear = u_angle_disappear.real;
      offset += sizeof(this->angle_disappear);
      union {
        int64_t real;
        uint64_t base;
      } u_mask_border;
      u_mask_border.base = 0;
      u_mask_border.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mask_border.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mask_border.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mask_border.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mask_border.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mask_border.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mask_border.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mask_border.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mask_border = u_mask_border.real;
      offset += sizeof(this->mask_border);
     return offset;
    }

    const char * getType(){ return "visp_tracker/KltSettings"; };
    const char * getMD5(){ return "a9f61cd7210b4d3872b77b5d1101b830"; };

  };

}
#endif