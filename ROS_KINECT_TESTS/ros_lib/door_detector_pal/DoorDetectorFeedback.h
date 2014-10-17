#ifndef _ROS_door_detector_pal_DoorDetectorFeedback_h
#define _ROS_door_detector_pal_DoorDetectorFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace door_detector_pal
{

  class DoorDetectorFeedback : public ros::Msg
  {
    public:
      char * comment;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_comment = strlen( (const char*) this->comment);
      memcpy(outbuffer + offset, &length_comment, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->comment, length_comment);
      offset += length_comment;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_comment;
      memcpy(&length_comment, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_comment; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_comment-1]=0;
      this->comment = (char *)(inbuffer + offset-1);
      offset += length_comment;
     return offset;
    }

    const char * getType(){ return "door_detector_pal/DoorDetectorFeedback"; };
    const char * getMD5(){ return "83609817d68993e8cc8571226bf4197d"; };

  };

}
#endif