#ifndef _ROS_SERVICE_GetInstalledBehaviors_h
#define _ROS_SERVICE_GetInstalledBehaviors_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace nao_msgs
{

static const char GETINSTALLEDBEHAVIORS[] = "nao_msgs/GetInstalledBehaviors";

  class GetInstalledBehaviorsRequest : public ros::Msg
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

    const char * getType(){ return GETINSTALLEDBEHAVIORS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetInstalledBehaviorsResponse : public ros::Msg
  {
    public:
      uint8_t behaviors_length;
      char* st_behaviors;
      char* * behaviors;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = behaviors_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < behaviors_length; i++){
      uint32_t length_behaviorsi = strlen( (const char*) this->behaviors[i]);
      memcpy(outbuffer + offset, &length_behaviorsi, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->behaviors[i], length_behaviorsi);
      offset += length_behaviorsi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t behaviors_lengthT = *(inbuffer + offset++);
      if(behaviors_lengthT > behaviors_length)
        this->behaviors = (char**)realloc(this->behaviors, behaviors_lengthT * sizeof(char*));
      offset += 3;
      behaviors_length = behaviors_lengthT;
      for( uint8_t i = 0; i < behaviors_length; i++){
      uint32_t length_st_behaviors;
      memcpy(&length_st_behaviors, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_behaviors; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_behaviors-1]=0;
      this->st_behaviors = (char *)(inbuffer + offset-1);
      offset += length_st_behaviors;
        memcpy( &(this->behaviors[i]), &(this->st_behaviors), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return GETINSTALLEDBEHAVIORS; };
    const char * getMD5(){ return "715783c8c6eb28fc2e1c05184add75ec"; };

  };

  class GetInstalledBehaviors {
    public:
    typedef GetInstalledBehaviorsRequest Request;
    typedef GetInstalledBehaviorsResponse Response;
  };

}
#endif
