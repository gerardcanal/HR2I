#ifndef _ROS_SERVICE_checkObjects_h
#define _ROS_SERVICE_checkObjects_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace nao_shopping_list
{

static const char CHECKOBJECTS[] = "nao_shopping_list/checkObjects";

  class checkObjectsRequest : public ros::Msg
  {
    public:

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return CHECKOBJECTS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class checkObjectsResponse : public ros::Msg
  {
    public:
      int8_t num_objects;
      uint8_t ids_length;
      int8_t st_ids;
      int8_t * ids;
      uint8_t names_length;
      char* st_names;
      char* * names;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_num_objects;
      u_num_objects.real = this->num_objects;
      *(outbuffer + offset + 0) = (u_num_objects.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->num_objects);
      *(outbuffer + offset++) = ids_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < ids_length; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_idsi;
      u_idsi.real = this->ids[i];
      *(outbuffer + offset + 0) = (u_idsi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ids[i]);
      }
      *(outbuffer + offset++) = names_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < names_length; i++){
      uint32_t length_namesi = strlen(this->names[i]);
      memcpy(outbuffer + offset, &length_namesi, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->names[i], length_namesi);
      offset += length_namesi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_num_objects;
      u_num_objects.base = 0;
      u_num_objects.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->num_objects = u_num_objects.real;
      offset += sizeof(this->num_objects);
      uint8_t ids_lengthT = *(inbuffer + offset++);
      if(ids_lengthT > ids_length)
        this->ids = (int8_t*)realloc(this->ids, ids_lengthT * sizeof(int8_t));
      offset += 3;
      ids_length = ids_lengthT;
      for( uint8_t i = 0; i < ids_length; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_st_ids;
      u_st_ids.base = 0;
      u_st_ids.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_ids = u_st_ids.real;
      offset += sizeof(this->st_ids);
        memcpy( &(this->ids[i]), &(this->st_ids), sizeof(int8_t));
      }
      uint8_t names_lengthT = *(inbuffer + offset++);
      if(names_lengthT > names_length)
        this->names = (char**)realloc(this->names, names_lengthT * sizeof(char*));
      offset += 3;
      names_length = names_lengthT;
      for( uint8_t i = 0; i < names_length; i++){
      uint32_t length_st_names;
      memcpy(&length_st_names, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_names-1]=0;
      this->st_names = (char *)(inbuffer + offset-1);
      offset += length_st_names;
        memcpy( &(this->names[i]), &(this->st_names), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return CHECKOBJECTS; };
    const char * getMD5(){ return "6fc7dae4228dae79ce50f63f9a5d490c"; };

  };

  class checkObjects {
    public:
    typedef checkObjectsRequest Request;
    typedef checkObjectsResponse Response;
  };

}
#endif
