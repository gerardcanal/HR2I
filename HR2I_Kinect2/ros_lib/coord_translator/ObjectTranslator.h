#ifndef _ROS_SERVICE_ObjectTranslator_h
#define _ROS_SERVICE_ObjectTranslator_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point32.h"

namespace coord_translator
{

static const char OBJECTTRANSLATOR[] = "coord_translator/ObjectTranslator";

  class ObjectTranslatorRequest : public ros::Msg
  {
    public:
      char * objname;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_objname = strlen( (const char*) this->objname);
      memcpy(outbuffer + offset, &length_objname, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->objname, length_objname);
      offset += length_objname;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_objname;
      memcpy(&length_objname, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_objname; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_objname-1]=0;
      this->objname = (char *)(inbuffer + offset-1);
      offset += length_objname;
     return offset;
    }

    const char * getType(){ return OBJECTTRANSLATOR; };
    const char * getMD5(){ return "c6eb39263756bf59242c791f46749c0f"; };

  };

  class ObjectTranslatorResponse : public ros::Msg
  {
    public:
      bool exists;
      char * category;
      geometry_msgs::Point32 base_coordinates;
      geometry_msgs::Pose arm_coordinates;
      int32_t databaseID;

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
      uint32_t length_category = strlen( (const char*) this->category);
      memcpy(outbuffer + offset, &length_category, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->category, length_category);
      offset += length_category;
      offset += this->base_coordinates.serialize(outbuffer + offset);
      offset += this->arm_coordinates.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_databaseID;
      u_databaseID.real = this->databaseID;
      *(outbuffer + offset + 0) = (u_databaseID.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_databaseID.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_databaseID.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_databaseID.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->databaseID);
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
      uint32_t length_category;
      memcpy(&length_category, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_category; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_category-1]=0;
      this->category = (char *)(inbuffer + offset-1);
      offset += length_category;
      offset += this->base_coordinates.deserialize(inbuffer + offset);
      offset += this->arm_coordinates.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_databaseID;
      u_databaseID.base = 0;
      u_databaseID.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_databaseID.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_databaseID.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_databaseID.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->databaseID = u_databaseID.real;
      offset += sizeof(this->databaseID);
     return offset;
    }

    const char * getType(){ return OBJECTTRANSLATOR; };
    const char * getMD5(){ return "6d8104347d7e8516cb29e102a5348b7b"; };

  };

  class ObjectTranslator {
    public:
    typedef ObjectTranslatorRequest Request;
    typedef ObjectTranslatorResponse Response;
  };

}
#endif
