#ifndef _ROS_face_recognition_FaceRecognitionActionFeedback_h
#define _ROS_face_recognition_FaceRecognitionActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "face_recognition/FaceRecognitionFeedback.h"

namespace face_recognition
{

  class FaceRecognitionActionFeedback : public ros::Msg
  {
    public:
      std_msgs::Header header;
      actionlib_msgs::GoalStatus status;
      face_recognition::FaceRecognitionFeedback feedback;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "face_recognition/FaceRecognitionActionFeedback"; };
    const char * getMD5(){ return "09c43b79bb624f9ee31bc35a9acf412e"; };

  };

}
#endif