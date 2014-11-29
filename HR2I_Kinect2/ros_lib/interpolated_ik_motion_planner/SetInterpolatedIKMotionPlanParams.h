#ifndef _ROS_SERVICE_SetInterpolatedIKMotionPlanParams_h
#define _ROS_SERVICE_SetInterpolatedIKMotionPlanParams_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace interpolated_ik_motion_planner
{

static const char SETINTERPOLATEDIKMOTIONPLANPARAMS[] = "interpolated_ik_motion_planner/SetInterpolatedIKMotionPlanParams";

  class SetInterpolatedIKMotionPlanParamsRequest : public ros::Msg
  {
    public:
      int32_t num_steps;
      double consistent_angle;
      int32_t collision_check_resolution;
      int32_t steps_before_abort;
      double pos_spacing;
      double rot_spacing;
      int8_t collision_aware;
      int8_t start_from_end;
      uint8_t max_joint_vels_length;
      double st_max_joint_vels;
      double * max_joint_vels;
      uint8_t max_joint_accs_length;
      double st_max_joint_accs;
      double * max_joint_accs;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_num_steps;
      u_num_steps.real = this->num_steps;
      *(outbuffer + offset + 0) = (u_num_steps.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_num_steps.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_num_steps.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_num_steps.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->num_steps);
      union {
        double real;
        uint64_t base;
      } u_consistent_angle;
      u_consistent_angle.real = this->consistent_angle;
      *(outbuffer + offset + 0) = (u_consistent_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_consistent_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_consistent_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_consistent_angle.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_consistent_angle.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_consistent_angle.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_consistent_angle.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_consistent_angle.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->consistent_angle);
      union {
        int32_t real;
        uint32_t base;
      } u_collision_check_resolution;
      u_collision_check_resolution.real = this->collision_check_resolution;
      *(outbuffer + offset + 0) = (u_collision_check_resolution.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_collision_check_resolution.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_collision_check_resolution.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_collision_check_resolution.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->collision_check_resolution);
      union {
        int32_t real;
        uint32_t base;
      } u_steps_before_abort;
      u_steps_before_abort.real = this->steps_before_abort;
      *(outbuffer + offset + 0) = (u_steps_before_abort.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_steps_before_abort.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_steps_before_abort.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_steps_before_abort.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->steps_before_abort);
      union {
        double real;
        uint64_t base;
      } u_pos_spacing;
      u_pos_spacing.real = this->pos_spacing;
      *(outbuffer + offset + 0) = (u_pos_spacing.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pos_spacing.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pos_spacing.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pos_spacing.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_pos_spacing.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_pos_spacing.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_pos_spacing.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_pos_spacing.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->pos_spacing);
      union {
        double real;
        uint64_t base;
      } u_rot_spacing;
      u_rot_spacing.real = this->rot_spacing;
      *(outbuffer + offset + 0) = (u_rot_spacing.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rot_spacing.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rot_spacing.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rot_spacing.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_rot_spacing.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_rot_spacing.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_rot_spacing.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_rot_spacing.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->rot_spacing);
      union {
        int8_t real;
        uint8_t base;
      } u_collision_aware;
      u_collision_aware.real = this->collision_aware;
      *(outbuffer + offset + 0) = (u_collision_aware.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->collision_aware);
      union {
        int8_t real;
        uint8_t base;
      } u_start_from_end;
      u_start_from_end.real = this->start_from_end;
      *(outbuffer + offset + 0) = (u_start_from_end.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->start_from_end);
      *(outbuffer + offset++) = max_joint_vels_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < max_joint_vels_length; i++){
      union {
        double real;
        uint64_t base;
      } u_max_joint_velsi;
      u_max_joint_velsi.real = this->max_joint_vels[i];
      *(outbuffer + offset + 0) = (u_max_joint_velsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_joint_velsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_joint_velsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_joint_velsi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_joint_velsi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_joint_velsi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_joint_velsi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_joint_velsi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_joint_vels[i]);
      }
      *(outbuffer + offset++) = max_joint_accs_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < max_joint_accs_length; i++){
      union {
        double real;
        uint64_t base;
      } u_max_joint_accsi;
      u_max_joint_accsi.real = this->max_joint_accs[i];
      *(outbuffer + offset + 0) = (u_max_joint_accsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_joint_accsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_joint_accsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_joint_accsi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_joint_accsi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_joint_accsi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_joint_accsi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_joint_accsi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_joint_accs[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_num_steps;
      u_num_steps.base = 0;
      u_num_steps.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_num_steps.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_num_steps.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_num_steps.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->num_steps = u_num_steps.real;
      offset += sizeof(this->num_steps);
      union {
        double real;
        uint64_t base;
      } u_consistent_angle;
      u_consistent_angle.base = 0;
      u_consistent_angle.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_consistent_angle.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_consistent_angle.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_consistent_angle.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_consistent_angle.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_consistent_angle.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_consistent_angle.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_consistent_angle.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->consistent_angle = u_consistent_angle.real;
      offset += sizeof(this->consistent_angle);
      union {
        int32_t real;
        uint32_t base;
      } u_collision_check_resolution;
      u_collision_check_resolution.base = 0;
      u_collision_check_resolution.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_collision_check_resolution.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_collision_check_resolution.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_collision_check_resolution.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->collision_check_resolution = u_collision_check_resolution.real;
      offset += sizeof(this->collision_check_resolution);
      union {
        int32_t real;
        uint32_t base;
      } u_steps_before_abort;
      u_steps_before_abort.base = 0;
      u_steps_before_abort.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_steps_before_abort.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_steps_before_abort.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_steps_before_abort.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->steps_before_abort = u_steps_before_abort.real;
      offset += sizeof(this->steps_before_abort);
      union {
        double real;
        uint64_t base;
      } u_pos_spacing;
      u_pos_spacing.base = 0;
      u_pos_spacing.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pos_spacing.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pos_spacing.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pos_spacing.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_pos_spacing.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_pos_spacing.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_pos_spacing.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_pos_spacing.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->pos_spacing = u_pos_spacing.real;
      offset += sizeof(this->pos_spacing);
      union {
        double real;
        uint64_t base;
      } u_rot_spacing;
      u_rot_spacing.base = 0;
      u_rot_spacing.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rot_spacing.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rot_spacing.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rot_spacing.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_rot_spacing.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_rot_spacing.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_rot_spacing.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_rot_spacing.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->rot_spacing = u_rot_spacing.real;
      offset += sizeof(this->rot_spacing);
      union {
        int8_t real;
        uint8_t base;
      } u_collision_aware;
      u_collision_aware.base = 0;
      u_collision_aware.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->collision_aware = u_collision_aware.real;
      offset += sizeof(this->collision_aware);
      union {
        int8_t real;
        uint8_t base;
      } u_start_from_end;
      u_start_from_end.base = 0;
      u_start_from_end.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->start_from_end = u_start_from_end.real;
      offset += sizeof(this->start_from_end);
      uint8_t max_joint_vels_lengthT = *(inbuffer + offset++);
      if(max_joint_vels_lengthT > max_joint_vels_length)
        this->max_joint_vels = (double*)realloc(this->max_joint_vels, max_joint_vels_lengthT * sizeof(double));
      offset += 3;
      max_joint_vels_length = max_joint_vels_lengthT;
      for( uint8_t i = 0; i < max_joint_vels_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_max_joint_vels;
      u_st_max_joint_vels.base = 0;
      u_st_max_joint_vels.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_max_joint_vels.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_max_joint_vels.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_max_joint_vels.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_max_joint_vels.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_max_joint_vels.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_max_joint_vels.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_max_joint_vels.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_max_joint_vels = u_st_max_joint_vels.real;
      offset += sizeof(this->st_max_joint_vels);
        memcpy( &(this->max_joint_vels[i]), &(this->st_max_joint_vels), sizeof(double));
      }
      uint8_t max_joint_accs_lengthT = *(inbuffer + offset++);
      if(max_joint_accs_lengthT > max_joint_accs_length)
        this->max_joint_accs = (double*)realloc(this->max_joint_accs, max_joint_accs_lengthT * sizeof(double));
      offset += 3;
      max_joint_accs_length = max_joint_accs_lengthT;
      for( uint8_t i = 0; i < max_joint_accs_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_max_joint_accs;
      u_st_max_joint_accs.base = 0;
      u_st_max_joint_accs.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_max_joint_accs.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_max_joint_accs.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_max_joint_accs.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_max_joint_accs.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_max_joint_accs.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_max_joint_accs.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_max_joint_accs.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_max_joint_accs = u_st_max_joint_accs.real;
      offset += sizeof(this->st_max_joint_accs);
        memcpy( &(this->max_joint_accs[i]), &(this->st_max_joint_accs), sizeof(double));
      }
     return offset;
    }

    const char * getType(){ return SETINTERPOLATEDIKMOTIONPLANPARAMS; };
    const char * getMD5(){ return "351122754b3043b9f5602d68d4eec5db"; };

  };

  class SetInterpolatedIKMotionPlanParamsResponse : public ros::Msg
  {
    public:

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return SETINTERPOLATEDIKMOTIONPLANPARAMS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetInterpolatedIKMotionPlanParams {
    public:
    typedef SetInterpolatedIKMotionPlanParamsRequest Request;
    typedef SetInterpolatedIKMotionPlanParamsResponse Response;
  };

}
#endif
