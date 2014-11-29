#ifndef _ROS_door_detector_pal_DoorDetectorActionGoal_h
#define _ROS_door_detector_pal_DoorDetectorActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "door_detector_pal/DoorDetectorGoal.h"

namespace door_detector_pal
{

  class DoorDetectorActionGoal : public ros::Msg
  {
    public:
      std_msgs::Header header;
      actionlib_msgs::GoalID goal_id;
      door_detector_pal::DoorDetectorGoal goal;

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

    const char * getType(){ return "door_detector_pal/DoorDetectorActionGoal"; };
    const char * getMD5(){ return "b5a58e3b24f820b81b1418234a646da0"; };

  };

}
#endif