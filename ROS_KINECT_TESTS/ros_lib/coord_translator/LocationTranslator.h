#ifndef _ROS_SERVICE_LocationTranslator_h
#define _ROS_SERVICE_LocationTranslator_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point32.h"

namespace coord_translator
{

static const char LOCATIONTRANSLATOR[] = "coord_translator/LocationTranslator";

  class LocationTranslatorRequest : public ros::Msg
  {
    public:
      char * location;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_location = strlen( (const char*) this->location);
      memcpy(outbuffer + offset, &length_location, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->location, length_location);
      offset += length_location;
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
     return offset;
    }

    const char * getType(){ return LOCATIONTRANSLATOR; };
    const char * getMD5(){ return "03da474bc61cfeb81a8854b4ca05bafa"; };

  };

  class LocationTranslatorResponse : public ros::Msg
  {
    public:
      bool exists;
      char * submap;
      geometry_msgs::Point32 coordinates;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_exists;
      u_exists.real = this->exists;
      *(outbuffer + offset + 0) = (u_exists.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->exists);
      uint32_t length_submap = strlen( (const char*) this->submap);
      memcpy(outbuffer + offset, &length_submap, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->submap, length_submap);
      offset += length_submap;
      offset += this->coordinates.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_exists;
      u_exists.base = 0;
      u_exists.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->exists = u_exists.real;
      offset += sizeof(this->exists);
      uint32_t length_submap;
      memcpy(&length_submap, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_submap; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_submap-1]=0;
      this->submap = (char *)(inbuffer + offset-1);
      offset += length_submap;
      offset += this->coordinates.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return LOCATIONTRANSLATOR; };
    const char * getMD5(){ return "c31816d2b637d2b7696f5dcff4faaf45"; };

  };

  class LocationTranslator {
    public:
    typedef LocationTranslatorRequest Request;
    typedef LocationTranslatorResponse Response;
  };

}
#endif
