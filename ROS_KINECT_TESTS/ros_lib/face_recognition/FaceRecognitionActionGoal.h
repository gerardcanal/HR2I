#ifndef _ROS_face_recognition_FaceRecognitionActionGoal_h
#define _ROS_face_recognition_FaceRecognitionActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "face_recognition/FaceRecognitionGoal.h"

namespace face_recognition
{

  class FaceRecognitionActionGoal : public ros::Msg
  {
    public:
      std_msgs::Header header;
      actionlib_msgs::GoalID goal_id;
      face_recognition::FaceRecognitionGoal goal;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->goal_id.serialize(outbuffer + offset);
      offset += this->goal.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->goal_id.deserialize(inbuffer + offset);
      offset += this->goal.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "face_recognition/FaceRecognitionActionGoal"; };
    const char * getMD5(){ return "4b046671794585c575f6b529dd4decb9"; };

  };

}
#endif