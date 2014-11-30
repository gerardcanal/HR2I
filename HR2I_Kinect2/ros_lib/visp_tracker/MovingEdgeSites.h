#ifndef _ROS_visp_tracker_MovingEdgeSites_h
#define _ROS_visp_tracker_MovingEdgeSites_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "visp_tracker/MovingEdgeSite.h"

namespace visp_tracker
{

  class MovingEdgeSites : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t moving_edge_sites_length;
      visp_tracker::MovingEdgeSite st_moving_edge_sites;
      visp_tracker::MovingEdgeSite * moving_edge_sites;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = moving_edge_sites_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < moving_edge_sites_length; i++){
      offset += this->moving_edge_sites[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t moving_edge_sites_lengthT = *(inbuffer + offset++);
      if(moving_edge_sites_lengthT > moving_edge_sites_length)
        this->moving_edge_sites = (visp_tracker::MovingEdgeSite*)realloc(this->moving_edge_sites, moving_edge_sites_lengthT * sizeof(visp_tracker::MovingEdgeSite));
      offset += 3;
      moving_edge_sites_length = moving_edge_sites_lengthT;
      for( uint8_t i = 0; i < moving_edge_sites_length; i++){
      offset += this->st_moving_edge_sites.deserialize(inbuffer + offset);
        memcpy( &(this->moving_edge_sites[i]), &(this->st_moving_edge_sites), sizeof(visp_tracker::MovingEdgeSite));
      }
     return offset;
    }

    const char * getType(){ return "visp_tracker/MovingEdgeSites"; };
    const char * getMD5(){ return "5293e8601467590a0dabbb34da47310c"; };

  };

}
#endif