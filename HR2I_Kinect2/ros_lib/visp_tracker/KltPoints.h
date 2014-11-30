#ifndef _ROS_visp_tracker_KltPoints_h
#define _ROS_visp_tracker_KltPoints_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "visp_tracker/KltPoint.h"

namespace visp_tracker
{

  class KltPoints : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t klt_points_positions_length;
      visp_tracker::KltPoint st_klt_points_positions;
      visp_tracker::KltPoint * klt_points_positions;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = klt_points_positions_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < klt_points_positions_length; i++){
      offset += this->klt_points_positions[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t klt_points_positions_lengthT = *(inbuffer + offset++);
      if(klt_points_positions_lengthT > klt_points_positions_length)
        this->klt_points_positions = (visp_tracker::KltPoint*)realloc(this->klt_points_positions, klt_points_positions_lengthT * sizeof(visp_tracker::KltPoint));
      offset += 3;
      klt_points_positions_length = klt_points_positions_lengthT;
      for( uint8_t i = 0; i < klt_points_positions_length; i++){
      offset += this->st_klt_points_positions.deserialize(inbuffer + offset);
        memcpy( &(this->klt_points_positions[i]), &(this->st_klt_points_positions), sizeof(visp_tracker::KltPoint));
      }
     return offset;
    }

    const char * getType(){ return "visp_tracker/KltPoints"; };
    const char * getMD5(){ return "681548d0f72044b7f086e3985d86f93c"; };

  };

}
#endif