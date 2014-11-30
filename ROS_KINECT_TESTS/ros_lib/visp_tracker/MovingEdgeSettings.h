#ifndef _ROS_visp_tracker_MovingEdgeSettings_h
#define _ROS_visp_tracker_MovingEdgeSettings_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace visp_tracker
{

  class MovingEdgeSettings : public ros::Msg
  {
    public:
      int64_t mask_size;
      int64_t n_mask;
      int64_t range;
      double threshold;
      double mu1;
      double mu2;
      int64_t sample_step;
      int64_t ntotal_sample;
      int64_t strip;
      double min_samplestep;
      double aberration;
      double init_aberration;
      double lambda;
      double first_threshold;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_mask_size;
      u_mask_size.real = this->mask_size;
      *(outbuffer + offset + 0) = (u_mask_size.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mask_size.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mask_size.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mask_size.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mask_size.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mask_size.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mask_size.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mask_size.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mask_size);
      union {
        int64_t real;
        uint64_t base;
      } u_n_mask;
      u_n_mask.real = this->n_mask;
      *(outbuffer + offset + 0) = (u_n_mask.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_n_mask.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_n_mask.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_n_mask.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_n_mask.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_n_mask.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_n_mask.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_n_mask.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->n_mask);
      union {
        int64_t real;
        uint64_t base;
      } u_range;
      u_range.real = this->range;
      *(outbuffer + offset + 0) = (u_range.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_range.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_range.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_range.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_range.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_range.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_range.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_range.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->range);
      union {
        double real;
        uint64_t base;
      } u_threshold;
      u_threshold.real = this->threshold;
      *(outbuffer + offset + 0) = (u_threshold.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_threshold.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_threshold.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_threshold.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_threshold.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_threshold.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_threshold.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_threshold.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->threshold);
      union {
        double real;
        uint64_t base;
      } u_mu1;
      u_mu1.real = this->mu1;
      *(outbuffer + offset + 0) = (u_mu1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mu1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mu1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mu1.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mu1.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mu1.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mu1.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mu1.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mu1);
      union {
        double real;
        uint64_t base;
      } u_mu2;
      u_mu2.real = this->mu2;
      *(outbuffer + offset + 0) = (u_mu2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mu2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mu2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mu2.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mu2.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mu2.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mu2.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mu2.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mu2);
      union {
        int64_t real;
        uint64_t base;
      } u_sample_step;
      u_sample_step.real = this->sample_step;
      *(outbuffer + offset + 0) = (u_sample_step.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sample_step.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sample_step.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sample_step.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_sample_step.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_sample_step.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_sample_step.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_sample_step.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->sample_step);
      union {
        int64_t real;
        uint64_t base;
      } u_ntotal_sample;
      u_ntotal_sample.real = this->ntotal_sample;
      *(outbuffer + offset + 0) = (u_ntotal_sample.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ntotal_sample.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ntotal_sample.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ntotal_sample.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_ntotal_sample.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_ntotal_sample.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_ntotal_sample.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_ntotal_sample.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->ntotal_sample);
      union {
        int64_t real;
        uint64_t base;
      } u_strip;
      u_strip.real = this->strip;
      *(outbuffer + offset + 0) = (u_strip.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_strip.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_strip.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_strip.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_strip.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_strip.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_strip.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_strip.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->strip);
      union {
        double real;
        uint64_t base;
      } u_min_samplestep;
      u_min_samplestep.real = this->min_samplestep;
      *(outbuffer + offset + 0) = (u_min_samplestep.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_samplestep.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_samplestep.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_samplestep.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_min_samplestep.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_min_samplestep.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_min_samplestep.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_min_samplestep.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->min_samplestep);
      union {
        double real;
        uint64_t base;
      } u_aberration;
      u_aberration.real = this->aberration;
      *(outbuffer + offset + 0) = (u_aberration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_aberration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_aberration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_aberration.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_aberration.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_aberration.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_aberration.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_aberration.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->aberration);
      union {
        double real;
        uint64_t base;
      } u_init_aberration;
      u_init_aberration.real = this->init_aberration;
      *(outbuffer + offset + 0) = (u_init_aberration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_init_aberration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_init_aberration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_init_aberration.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_init_aberration.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_init_aberration.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_init_aberration.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_init_aberration.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->init_aberration);
      union {
        double real;
        uint64_t base;
      } u_lambda;
      u_lambda.real = this->lambda;
      *(outbuffer + offset + 0) = (u_lambda.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lambda.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lambda.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lambda.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_lambda.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_lambda.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_lambda.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_lambda.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->lambda);
      union {
        double real;
        uint64_t base;
      } u_first_threshold;
      u_first_threshold.real = this->first_threshold;
      *(outbuffer + offset + 0) = (u_first_threshold.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_first_threshold.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_first_threshold.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_first_threshold.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_first_threshold.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_first_threshold.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_first_threshold.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_first_threshold.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->first_threshold);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_mask_size;
      u_mask_size.base = 0;
      u_mask_size.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mask_size.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mask_size.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mask_size.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mask_size.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mask_size.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mask_size.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mask_size.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mask_size = u_mask_size.real;
      offset += sizeof(this->mask_size);
      union {
        int64_t real;
        uint64_t base;
      } u_n_mask;
      u_n_mask.base = 0;
      u_n_mask.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_n_mask.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_n_mask.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_n_mask.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_n_mask.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_n_mask.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_n_mask.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_n_mask.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->n_mask = u_n_mask.real;
      offset += sizeof(this->n_mask);
      union {
        int64_t real;
        uint64_t base;
      } u_range;
      u_range.base = 0;
      u_range.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_range.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_range.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_range.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_range.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_range.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_range.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_range.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->range = u_range.real;
      offset += sizeof(this->range);
      union {
        double real;
        uint64_t base;
      } u_threshold;
      u_threshold.base = 0;
      u_threshold.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_threshold.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_threshold.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_threshold.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_threshold.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_threshold.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_threshold.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_threshold.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->threshold = u_threshold.real;
      offset += sizeof(this->threshold);
      union {
        double real;
        uint64_t base;
      } u_mu1;
      u_mu1.base = 0;
      u_mu1.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mu1.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mu1.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mu1.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mu1.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mu1.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mu1.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mu1.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mu1 = u_mu1.real;
      offset += sizeof(this->mu1);
      union {
        double real;
        uint64_t base;
      } u_mu2;
      u_mu2.base = 0;
      u_mu2.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mu2.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mu2.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mu2.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mu2.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mu2.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mu2.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mu2.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mu2 = u_mu2.real;
      offset += sizeof(this->mu2);
      union {
        int64_t real;
        uint64_t base;
      } u_sample_step;
      u_sample_step.base = 0;
      u_sample_step.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sample_step.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sample_step.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sample_step.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_sample_step.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_sample_step.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_sample_step.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_sample_step.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->sample_step = u_sample_step.real;
      offset += sizeof(this->sample_step);
      union {
        int64_t real;
        uint64_t base;
      } u_ntotal_sample;
      u_ntotal_sample.base = 0;
      u_ntotal_sample.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ntotal_sample.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ntotal_sample.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ntotal_sample.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_ntotal_sample.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_ntotal_sample.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_ntotal_sample.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_ntotal_sample.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->ntotal_sample = u_ntotal_sample.real;
      offset += sizeof(this->ntotal_sample);
      union {
        int64_t real;
        uint64_t base;
      } u_strip;
      u_strip.base = 0;
      u_strip.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_strip.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_strip.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_strip.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_strip.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_strip.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_strip.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_strip.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->strip = u_strip.real;
      offset += sizeof(this->strip);
      union {
        double real;
        uint64_t base;
      } u_min_samplestep;
      u_min_samplestep.base = 0;
      u_min_samplestep.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_samplestep.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_samplestep.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_samplestep.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_min_samplestep.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_min_samplestep.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_min_samplestep.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_min_samplestep.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->min_samplestep = u_min_samplestep.real;
      offset += sizeof(this->min_samplestep);
      union {
        double real;
        uint64_t base;
      } u_aberration;
      u_aberration.base = 0;
      u_aberration.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_aberration.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_aberration.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_aberration.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_aberration.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_aberration.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_aberration.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_aberration.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->aberration = u_aberration.real;
      offset += sizeof(this->aberration);
      union {
        double real;
        uint64_t base;
      } u_init_aberration;
      u_init_aberration.base = 0;
      u_init_aberration.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_init_aberration.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_init_aberration.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_init_aberration.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_init_aberration.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_init_aberration.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_init_aberration.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_init_aberration.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->init_aberration = u_init_aberration.real;
      offset += sizeof(this->init_aberration);
      union {
        double real;
        uint64_t base;
      } u_lambda;
      u_lambda.base = 0;
      u_lambda.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lambda.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lambda.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lambda.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_lambda.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_lambda.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_lambda.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_lambda.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->lambda = u_lambda.real;
      offset += sizeof(this->lambda);
      union {
        double real;
        uint64_t base;
      } u_first_threshold;
      u_first_threshold.base = 0;
      u_first_threshold.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_first_threshold.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_first_threshold.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_first_threshold.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_first_threshold.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_first_threshold.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_first_threshold.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_first_threshold.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->first_threshold = u_first_threshold.real;
      offset += sizeof(this->first_threshold);
     return offset;
    }

    const char * getType(){ return "visp_tracker/MovingEdgeSettings"; };
    const char * getMD5(){ return "376fefab194f3282c421288b8a099b76"; };

  };

}
#endif