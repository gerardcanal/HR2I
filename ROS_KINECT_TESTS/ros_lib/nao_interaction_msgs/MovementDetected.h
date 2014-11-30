#ifndef _ROS_nao_interaction_msgs_MovementDetected_h
#define _ROS_nao_interaction_msgs_MovementDetected_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"

namespace nao_interaction_msgs
{

  class MovementDetected : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::Point gravity_center;
      geometry_msgs::Point mean_velocity;
      uint8_t points_poses_length;
      geometry_msgs::Point st_points_poses;
      geometry_msgs::Point * points_poses;
      uint8_t points_speeds_length;
      geometry_msgs::Point st_points_speeds;
      geometry_msgs::Point * points_speeds;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->gravity_center.serialize(outbuffer + offset);
      offset += this->mean_velocity.serialize(outbuffer + offset);
      *(outbuffer + offset++) = points_poses_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < points_poses_length; i++){
      offset += this->points_poses[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = points_speeds_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < points_speeds_length; i++){
      offset += this->points_speeds[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->gravity_center.deserialize(inbuffer + offset);
      offset += this->mean_velocity.deserialize(inbuffer + offset);
      uint8_t points_poses_lengthT = *(inbuffer + offset++);
      if(points_poses_lengthT > points_poses_length)
        this->points_poses = (geometry_msgs::Point*)realloc(this->points_poses, points_poses_lengthT * sizeof(geometry_msgs::Point));
      offset += 3;
      points_poses_length = points_poses_lengthT;
      for( uint8_t i = 0; i < points_poses_length; i++){
      offset += this->st_points_poses.deserialize(inbuffer + offset);
        memcpy( &(this->points_poses[i]), &(this->st_points_poses), sizeof(geometry_msgs::Point));
      }
      uint8_t points_speeds_lengthT = *(inbuffer + offset++);
      if(points_speeds_lengthT > points_speeds_length)
        this->points_speeds = (geometry_msgs::Point*)realloc(this->points_speeds, points_speeds_lengthT * sizeof(geometry_msgs::Point));
      offset += 3;
      points_speeds_length = points_speeds_lengthT;
      for( uint8_t i = 0; i < points_speeds_length; i++){
      offset += this->st_points_speeds.deserialize(inbuffer + offset);
        memcpy( &(this->points_speeds[i]), &(this->st_points_speeds), sizeof(geometry_msgs::Point));
      }
     return offset;
    }

    const char * getType(){ return "nao_interaction_msgs/MovementDetected"; };
    const char * getMD5(){ return "ae24dbf38f441d6f1cae72eb224ecd17"; };

  };

}
#endif