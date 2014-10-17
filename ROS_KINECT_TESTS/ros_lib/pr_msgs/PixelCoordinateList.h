#ifndef _ROS_pr_msgs_PixelCoordinateList_h
#define _ROS_pr_msgs_PixelCoordinateList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "pr_msgs/PixelCoordinate.h"
#include "ros/time.h"

namespace pr_msgs
{

  class PixelCoordinateList : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t coordinates_length;
      pr_msgs::PixelCoordinate st_coordinates;
      pr_msgs::PixelCoordinate * coordinates;
      ros::Time originalTimeStamp;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = coordinates_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < coordinates_length; i++){
      offset += this->coordinates[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->originalTimeStamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->originalTimeStamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->originalTimeStamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->originalTimeStamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->originalTimeStamp.sec);
      *(outbuffer + offset + 0) = (this->originalTimeStamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->originalTimeStamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->originalTimeStamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->originalTimeStamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->originalTimeStamp.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t coordinates_lengthT = *(inbuffer + offset++);
      if(coordinates_lengthT > coordinates_length)
        this->coordinates = (pr_msgs::PixelCoordinate*)realloc(this->coordinates, coordinates_lengthT * sizeof(pr_msgs::PixelCoordinate));
      offset += 3;
      coordinates_length = coordinates_lengthT;
      for( uint8_t i = 0; i < coordinates_length; i++){
      offset += this->st_coordinates.deserialize(inbuffer + offset);
        memcpy( &(this->coordinates[i]), &(this->st_coordinates), sizeof(pr_msgs::PixelCoordinate));
      }
      this->originalTimeStamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->originalTimeStamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->originalTimeStamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->originalTimeStamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->originalTimeStamp.sec);
      this->originalTimeStamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->originalTimeStamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->originalTimeStamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->originalTimeStamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->originalTimeStamp.nsec);
     return offset;
    }

    const char * getType(){ return "pr_msgs/PixelCoordinateList"; };
    const char * getMD5(){ return "3c9bc0eadac36ef3a77fd4cd41e78c02"; };

  };

}
#endif