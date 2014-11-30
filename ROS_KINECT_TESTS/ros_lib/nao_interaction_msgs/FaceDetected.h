#ifndef _ROS_nao_interaction_msgs_FaceDetected_h
#define _ROS_nao_interaction_msgs_FaceDetected_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"

namespace nao_interaction_msgs
{

  class FaceDetected : public ros::Msg
  {
    public:
      std_msgs::Header header;
      std_msgs::Int32 face_id;
      std_msgs::Float32 score_reco;
      std_msgs::String face_label;
      std_msgs::Float32 shape_alpha;
      std_msgs::Float32 shape_beta;
      std_msgs::Float32 shape_sizeX;
      std_msgs::Float32 shape_sizeY;
      std_msgs::Float32 right_eye_eyeCenter_x;
      std_msgs::Float32 right_eye_eyeCenter_y;
      std_msgs::Float32 right_eye_noseSideLimit_x;
      std_msgs::Float32 right_eye_noseSideLimit_y;
      std_msgs::Float32 right_eye_earSideLimit_x;
      std_msgs::Float32 right_eye_earSideLimit_y;
      std_msgs::Float32 right_eye_topLimit_x;
      std_msgs::Float32 right_eye_topLimit_y;
      std_msgs::Float32 right_eye_bottomLimit_x;
      std_msgs::Float32 right_eye_bottomLimit_y;
      std_msgs::Float32 right_eye_midTopEarLimit_x;
      std_msgs::Float32 right_eye_midTopEarLimit_y;
      std_msgs::Float32 right_eye_midTopNoseLimit_x;
      std_msgs::Float32 right_eye_midTopNoseLimit_y;
      std_msgs::Float32 left_eye_eyeCenter_x;
      std_msgs::Float32 left_eye_eyeCenter_y;
      std_msgs::Float32 left_eye_noseSideLimit_x;
      std_msgs::Float32 left_eye_noseSideLimit_y;
      std_msgs::Float32 left_eye_earSideLimit_x;
      std_msgs::Float32 left_eye_earSideLimit_y;
      std_msgs::Float32 left_eye_topLimit_x;
      std_msgs::Float32 left_eye_topLimit_y;
      std_msgs::Float32 left_eye_bottomLimit_x;
      std_msgs::Float32 left_eye_bottomLimit_y;
      std_msgs::Float32 left_eye_midTopEarLimit_x;
      std_msgs::Float32 left_eye_midTopEarLimit_y;
      std_msgs::Float32 left_eye_midTopNoseLimit_x;
      std_msgs::Float32 left_eye_midTopNoseLimit_y;
      std_msgs::Float32 right_eyebrow_noseSideLimit_x;
      std_msgs::Float32 right_eyebrow_noseSideLimit_y;
      std_msgs::Float32 right_eyebrow_center_x;
      std_msgs::Float32 right_eyebrow_center_y;
      std_msgs::Float32 right_eyebrow_earSideLimit_x;
      std_msgs::Float32 right_eyebrow_earSideLimit_y;
      std_msgs::Float32 left_eyebrow_noseSideLimit_x;
      std_msgs::Float32 left_eyebrow_noseSideLimit_y;
      std_msgs::Float32 left_eyebrow_center_x;
      std_msgs::Float32 left_eyebrow_center_y;
      std_msgs::Float32 left_eyebrow_earSideLimit_x;
      std_msgs::Float32 left_eyebrow_earSideLimit_y;
      std_msgs::Float32 nose_bottomCenterLimit_x;
      std_msgs::Float32 nose_bottomCenterLimit_y;
      std_msgs::Float32 nose_bottomLeftLimit_x;
      std_msgs::Float32 nose_bottomLeftLimit_y;
      std_msgs::Float32 nose_bottomRightLimit_x;
      std_msgs::Float32 nose_bottomRightLimit_y;
      std_msgs::Float32 mouth_leftLimit_x;
      std_msgs::Float32 mouth_leftLimit_y;
      std_msgs::Float32 mouth_rightLimit_x;
      std_msgs::Float32 mouth_rightLimit_y;
      std_msgs::Float32 mouth_topLimit_x;
      std_msgs::Float32 mouth_topLimit_y;
      std_msgs::Float32 mouth_bottomLimit_x;
      std_msgs::Float32 mouth_bottomLimit_y;
      std_msgs::Float32 mouth_midTopLeftLimit_x;
      std_msgs::Float32 mouth_midTopLeftLimit_y;
      std_msgs::Float32 mouth_midTopRightLimit_x;
      std_msgs::Float32 mouth_midTopRightLimit_y;
      std_msgs::Float32 mouth_midBottomRightLimit_x;
      std_msgs::Float32 mouth_midBottomRightLimit_y;
      std_msgs::Float32 mouth_midBottomLeftLimit_x;
      std_msgs::Float32 mouth_midBottomLeftLimit_y;
      geometry_msgs::Pose camera_0_pose;
      geometry_msgs::Pose camera_1_pose;
      std_msgs::Int32 camera_id;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->face_id.serialize(outbuffer + offset);
      offset += this->score_reco.serialize(outbuffer + offset);
      offset += this->face_label.serialize(outbuffer + offset);
      offset += this->shape_alpha.serialize(outbuffer + offset);
      offset += this->shape_beta.serialize(outbuffer + offset);
      offset += this->shape_sizeX.serialize(outbuffer + offset);
      offset += this->shape_sizeY.serialize(outbuffer + offset);
      offset += this->right_eye_eyeCenter_x.serialize(outbuffer + offset);
      offset += this->right_eye_eyeCenter_y.serialize(outbuffer + offset);
      offset += this->right_eye_noseSideLimit_x.serialize(outbuffer + offset);
      offset += this->right_eye_noseSideLimit_y.serialize(outbuffer + offset);
      offset += this->right_eye_earSideLimit_x.serialize(outbuffer + offset);
      offset += this->right_eye_earSideLimit_y.serialize(outbuffer + offset);
      offset += this->right_eye_topLimit_x.serialize(outbuffer + offset);
      offset += this->right_eye_topLimit_y.serialize(outbuffer + offset);
      offset += this->right_eye_bottomLimit_x.serialize(outbuffer + offset);
      offset += this->right_eye_bottomLimit_y.serialize(outbuffer + offset);
      offset += this->right_eye_midTopEarLimit_x.serialize(outbuffer + offset);
      offset += this->right_eye_midTopEarLimit_y.serialize(outbuffer + offset);
      offset += this->right_eye_midTopNoseLimit_x.serialize(outbuffer + offset);
      offset += this->right_eye_midTopNoseLimit_y.serialize(outbuffer + offset);
      offset += this->left_eye_eyeCenter_x.serialize(outbuffer + offset);
      offset += this->left_eye_eyeCenter_y.serialize(outbuffer + offset);
      offset += this->left_eye_noseSideLimit_x.serialize(outbuffer + offset);
      offset += this->left_eye_noseSideLimit_y.serialize(outbuffer + offset);
      offset += this->left_eye_earSideLimit_x.serialize(outbuffer + offset);
      offset += this->left_eye_earSideLimit_y.serialize(outbuffer + offset);
      offset += this->left_eye_topLimit_x.serialize(outbuffer + offset);
      offset += this->left_eye_topLimit_y.serialize(outbuffer + offset);
      offset += this->left_eye_bottomLimit_x.serialize(outbuffer + offset);
      offset += this->left_eye_bottomLimit_y.serialize(outbuffer + offset);
      offset += this->left_eye_midTopEarLimit_x.serialize(outbuffer + offset);
      offset += this->left_eye_midTopEarLimit_y.serialize(outbuffer + offset);
      offset += this->left_eye_midTopNoseLimit_x.serialize(outbuffer + offset);
      offset += this->left_eye_midTopNoseLimit_y.serialize(outbuffer + offset);
      offset += this->right_eyebrow_noseSideLimit_x.serialize(outbuffer + offset);
      offset += this->right_eyebrow_noseSideLimit_y.serialize(outbuffer + offset);
      offset += this->right_eyebrow_center_x.serialize(outbuffer + offset);
      offset += this->right_eyebrow_center_y.serialize(outbuffer + offset);
      offset += this->right_eyebrow_earSideLimit_x.serialize(outbuffer + offset);
      offset += this->right_eyebrow_earSideLimit_y.serialize(outbuffer + offset);
      offset += this->left_eyebrow_noseSideLimit_x.serialize(outbuffer + offset);
      offset += this->left_eyebrow_noseSideLimit_y.serialize(outbuffer + offset);
      offset += this->left_eyebrow_center_x.serialize(outbuffer + offset);
      offset += this->left_eyebrow_center_y.serialize(outbuffer + offset);
      offset += this->left_eyebrow_earSideLimit_x.serialize(outbuffer + offset);
      offset += this->left_eyebrow_earSideLimit_y.serialize(outbuffer + offset);
      offset += this->nose_bottomCenterLimit_x.serialize(outbuffer + offset);
      offset += this->nose_bottomCenterLimit_y.serialize(outbuffer + offset);
      offset += this->nose_bottomLeftLimit_x.serialize(outbuffer + offset);
      offset += this->nose_bottomLeftLimit_y.serialize(outbuffer + offset);
      offset += this->nose_bottomRightLimit_x.serialize(outbuffer + offset);
      offset += this->nose_bottomRightLimit_y.serialize(outbuffer + offset);
      offset += this->mouth_leftLimit_x.serialize(outbuffer + offset);
      offset += this->mouth_leftLimit_y.serialize(outbuffer + offset);
      offset += this->mouth_rightLimit_x.serialize(outbuffer + offset);
      offset += this->mouth_rightLimit_y.serialize(outbuffer + offset);
      offset += this->mouth_topLimit_x.serialize(outbuffer + offset);
      offset += this->mouth_topLimit_y.serialize(outbuffer + offset);
      offset += this->mouth_bottomLimit_x.serialize(outbuffer + offset);
      offset += this->mouth_bottomLimit_y.serialize(outbuffer + offset);
      offset += this->mouth_midTopLeftLimit_x.serialize(outbuffer + offset);
      offset += this->mouth_midTopLeftLimit_y.serialize(outbuffer + offset);
      offset += this->mouth_midTopRightLimit_x.serialize(outbuffer + offset);
      offset += this->mouth_midTopRightLimit_y.serialize(outbuffer + offset);
      offset += this->mouth_midBottomRightLimit_x.serialize(outbuffer + offset);
      offset += this->mouth_midBottomRightLimit_y.serialize(outbuffer + offset);
      offset += this->mouth_midBottomLeftLimit_x.serialize(outbuffer + offset);
      offset += this->mouth_midBottomLeftLimit_y.serialize(outbuffer + offset);
      offset += this->camera_0_pose.serialize(outbuffer + offset);
      offset += this->camera_1_pose.serialize(outbuffer + offset);
      offset += this->camera_id.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->face_id.deserialize(inbuffer + offset);
      offset += this->score_reco.deserialize(inbuffer + offset);
      offset += this->face_label.deserialize(inbuffer + offset);
      offset += this->shape_alpha.deserialize(inbuffer + offset);
      offset += this->shape_beta.deserialize(inbuffer + offset);
      offset += this->shape_sizeX.deserialize(inbuffer + offset);
      offset += this->shape_sizeY.deserialize(inbuffer + offset);
      offset += this->right_eye_eyeCenter_x.deserialize(inbuffer + offset);
      offset += this->right_eye_eyeCenter_y.deserialize(inbuffer + offset);
      offset += this->right_eye_noseSideLimit_x.deserialize(inbuffer + offset);
      offset += this->right_eye_noseSideLimit_y.deserialize(inbuffer + offset);
      offset += this->right_eye_earSideLimit_x.deserialize(inbuffer + offset);
      offset += this->right_eye_earSideLimit_y.deserialize(inbuffer + offset);
      offset += this->right_eye_topLimit_x.deserialize(inbuffer + offset);
      offset += this->right_eye_topLimit_y.deserialize(inbuffer + offset);
      offset += this->right_eye_bottomLimit_x.deserialize(inbuffer + offset);
      offset += this->right_eye_bottomLimit_y.deserialize(inbuffer + offset);
      offset += this->right_eye_midTopEarLimit_x.deserialize(inbuffer + offset);
      offset += this->right_eye_midTopEarLimit_y.deserialize(inbuffer + offset);
      offset += this->right_eye_midTopNoseLimit_x.deserialize(inbuffer + offset);
      offset += this->right_eye_midTopNoseLimit_y.deserialize(inbuffer + offset);
      offset += this->left_eye_eyeCenter_x.deserialize(inbuffer + offset);
      offset += this->left_eye_eyeCenter_y.deserialize(inbuffer + offset);
      offset += this->left_eye_noseSideLimit_x.deserialize(inbuffer + offset);
      offset += this->left_eye_noseSideLimit_y.deserialize(inbuffer + offset);
      offset += this->left_eye_earSideLimit_x.deserialize(inbuffer + offset);
      offset += this->left_eye_earSideLimit_y.deserialize(inbuffer + offset);
      offset += this->left_eye_topLimit_x.deserialize(inbuffer + offset);
      offset += this->left_eye_topLimit_y.deserialize(inbuffer + offset);
      offset += this->left_eye_bottomLimit_x.deserialize(inbuffer + offset);
      offset += this->left_eye_bottomLimit_y.deserialize(inbuffer + offset);
      offset += this->left_eye_midTopEarLimit_x.deserialize(inbuffer + offset);
      offset += this->left_eye_midTopEarLimit_y.deserialize(inbuffer + offset);
      offset += this->left_eye_midTopNoseLimit_x.deserialize(inbuffer + offset);
      offset += this->left_eye_midTopNoseLimit_y.deserialize(inbuffer + offset);
      offset += this->right_eyebrow_noseSideLimit_x.deserialize(inbuffer + offset);
      offset += this->right_eyebrow_noseSideLimit_y.deserialize(inbuffer + offset);
      offset += this->right_eyebrow_center_x.deserialize(inbuffer + offset);
      offset += this->right_eyebrow_center_y.deserialize(inbuffer + offset);
      offset += this->right_eyebrow_earSideLimit_x.deserialize(inbuffer + offset);
      offset += this->right_eyebrow_earSideLimit_y.deserialize(inbuffer + offset);
      offset += this->left_eyebrow_noseSideLimit_x.deserialize(inbuffer + offset);
      offset += this->left_eyebrow_noseSideLimit_y.deserialize(inbuffer + offset);
      offset += this->left_eyebrow_center_x.deserialize(inbuffer + offset);
      offset += this->left_eyebrow_center_y.deserialize(inbuffer + offset);
      offset += this->left_eyebrow_earSideLimit_x.deserialize(inbuffer + offset);
      offset += this->left_eyebrow_earSideLimit_y.deserialize(inbuffer + offset);
      offset += this->nose_bottomCenterLimit_x.deserialize(inbuffer + offset);
      offset += this->nose_bottomCenterLimit_y.deserialize(inbuffer + offset);
      offset += this->nose_bottomLeftLimit_x.deserialize(inbuffer + offset);
      offset += this->nose_bottomLeftLimit_y.deserialize(inbuffer + offset);
      offset += this->nose_bottomRightLimit_x.deserialize(inbuffer + offset);
      offset += this->nose_bottomRightLimit_y.deserialize(inbuffer + offset);
      offset += this->mouth_leftLimit_x.deserialize(inbuffer + offset);
      offset += this->mouth_leftLimit_y.deserialize(inbuffer + offset);
      offset += this->mouth_rightLimit_x.deserialize(inbuffer + offset);
      offset += this->mouth_rightLimit_y.deserialize(inbuffer + offset);
      offset += this->mouth_topLimit_x.deserialize(inbuffer + offset);
      offset += this->mouth_topLimit_y.deserialize(inbuffer + offset);
      offset += this->mouth_bottomLimit_x.deserialize(inbuffer + offset);
      offset += this->mouth_bottomLimit_y.deserialize(inbuffer + offset);
      offset += this->mouth_midTopLeftLimit_x.deserialize(inbuffer + offset);
      offset += this->mouth_midTopLeftLimit_y.deserialize(inbuffer + offset);
      offset += this->mouth_midTopRightLimit_x.deserialize(inbuffer + offset);
      offset += this->mouth_midTopRightLimit_y.deserialize(inbuffer + offset);
      offset += this->mouth_midBottomRightLimit_x.deserialize(inbuffer + offset);
      offset += this->mouth_midBottomRightLimit_y.deserialize(inbuffer + offset);
      offset += this->mouth_midBottomLeftLimit_x.deserialize(inbuffer + offset);
      offset += this->mouth_midBottomLeftLimit_y.deserialize(inbuffer + offset);
      offset += this->camera_0_pose.deserialize(inbuffer + offset);
      offset += this->camera_1_pose.deserialize(inbuffer + offset);
      offset += this->camera_id.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "nao_interaction_msgs/FaceDetected"; };
    const char * getMD5(){ return "3b3868bc92bb74386686642b1cf73f53"; };

  };

}
#endif