#ifndef _ROS_door_detector_pal_DoorDetectorResult_h
#define _ROS_door_detector_pal_DoorDetectorResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"

namespace door_detector_pal
{

  class DoorDetectorResult : public ros::Msg
  {
    public:
      char * handle_side;
      char * door_status;
      geometry_msgs::PoseStamped door_handle;
      geometry_msgs::PoseStamped door_position;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_handle_side = strlen( (const char*) this->handle_side);
      memcpy(outbuffer + offset, &length_handle_side, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->handle_side, length_handle_side);
      offset += length_handle_side;
      uint32_t length_door_status = strlen( (const char*) this->door_status);
      memcpy(outbuffer + offset, &length_door_status, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->door_status, length_door_status);
      offset += length_door_status;
      offset += this->door_handle.serialize(outbuffer + offset);
      offset += this->door_position.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_handle_side;
      memcpy(&length_handle_side, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_handle_side; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_handle_side-1]=0;
      this->handle_side = (char *)(inbuffer + offset-1);
      offset += length_handle_side;
      uint32_t length_door_status;
      memcpy(&length_door_status, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_door_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_door_status-1]=0;
      this->door_status = (char *)(inbuffer + offset-1);
      offset += length_door_status;
      offset += this->door_handle.deserialize(inbuffer + offset);
      offset += this->door_position.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "door_detector_pal/DoorDetectorResult"; };
    const char * getMD5(){ return "5769294bcaaeb780153856a9d6238a27"; };

  };

}
#endif