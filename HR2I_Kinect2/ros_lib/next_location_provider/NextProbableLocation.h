#ifndef _ROS_SERVICE_NextProbableLocation_h
#define _ROS_SERVICE_NextProbableLocation_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace next_location_provider
{

static const char NEXTPROBABLELOCATION[] = "next_location_provider/NextProbableLocation";

  class NextProbableLocationRequest : public ros::Msg
  {
    public:
      char * room;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_room = strlen( (const char*) this->room);
      memcpy(outbuffer + offset, &length_room, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->room, length_room);
      offset += length_room;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_room;
      memcpy(&length_room, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_room; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_room-1]=0;
      this->room = (char *)(inbuffer + offset-1);
      offset += length_room;
     return offset;
    }

    const char * getType(){ return NEXTPROBABLELOCATION; };
    const char * getMD5(){ return "e497569192cccb82020c3a5c262721b9"; };

  };

  class NextProbableLocationResponse : public ros::Msg
  {
    public:
      char * location;
      geometry_msgs::Pose loc_position;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_location = strlen( (const char*) this->location);
      memcpy(outbuffer + offset, &length_location, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->location, length_location);
      offset += length_location;
      offset += this->loc_position.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_location;
      memcpy(&length_location, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_location; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_location-1]=0;
      this->location = (char *)(inbuffer + offset-1);
      offset += length_location;
      offset += this->loc_position.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return NEXTPROBABLELOCATION; };
    const char * getMD5(){ return "93bb4d064db971c98b3a2587cd6a32dc"; };

  };

  class NextProbableLocation {
    public:
    typedef NextProbableLocationRequest Request;
    typedef NextProbableLocationResponse Response;
  };

}
#endif
