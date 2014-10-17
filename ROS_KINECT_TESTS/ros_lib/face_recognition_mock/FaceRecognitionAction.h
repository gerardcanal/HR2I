#ifndef _ROS_face_recognition_mock_FaceRecognitionAction_h
#define _ROS_face_recognition_mock_FaceRecognitionAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "face_recognition_mock/FaceRecognitionActionGoal.h"
#include "face_recognition_mock/FaceRecognitionActionResult.h"
#include "face_recognition_mock/FaceRecognitionActionFeedback.h"

namespace face_recognition_mock
{

  class FaceRecognitionAction : public ros::Msg
  {
    public:
      face_recognition_mock::FaceRecognitionActionGoal action_goal;
      face_recognition_mock::FaceRecognitionActionResult action_result;
      face_recognition_mock::FaceRecognitionActionFeedback action_feedback;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "face_recognition_mock/FaceRecognitionAction"; };
    const char * getMD5(){ return "102b0392d6292ae2bbbc788be5a2bd52"; };

  };

}
#endif