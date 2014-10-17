#ifndef _ROS_face_recognition_FaceRecognitionActionResult_h
#define _ROS_face_recognition_FaceRecognitionActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "face_recognition/FaceRecognitionResult.h"

namespace face_recognition
{

  class FaceRecognitionActionResult : public ros::Msg
  {
    public:
      std_msgs::Header header;
      actionlib_msgs::GoalStatus status;
      face_recognition::FaceRecognitionResult result;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "face_recognition/FaceRecognitionActionResult"; };
    const char * getMD5(){ return "0c8a797b269880ab70a8762f40d4f4e2"; };

  };

}
#endif