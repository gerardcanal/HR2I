#ifndef _ROS_visp_hand2eye_calibration_TransformArray_h
#define _ROS_visp_hand2eye_calibration_TransformArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Transform.h"

namespace visp_hand2eye_calibration
{

  class TransformArray : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t transforms_length;
      geometry_msgs::Transform st_transforms;
      geometry_msgs::Transform * transforms;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = transforms_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < transforms_length; i++){
      offset += this->transforms[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t transforms_lengthT = *(inbuffer + offset++);
      if(transforms_lengthT > transforms_length)
        this->transforms = (geometry_msgs::Transform*)realloc(this->transforms, transforms_lengthT * sizeof(geometry_msgs::Transform));
      offset += 3;
      transforms_length = transforms_lengthT;
      for( uint8_t i = 0; i < transforms_length; i++){
      offset += this->st_transforms.deserialize(inbuffer + offset);
        memcpy( &(this->transforms[i]), &(this->st_transforms), sizeof(geometry_msgs::Transform));
      }
     return offset;
    }

    const char * getType(){ return "visp_hand2eye_calibration/TransformArray"; };
    const char * getMD5(){ return "69339633e969be70367b6ff0fe206328"; };

  };

}
#endif