#ifndef _ROS_pal_interaction_msgs_asrresult_h
#define _ROS_pal_interaction_msgs_asrresult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"
#include "pal_interaction_msgs/actiontag.h"

namespace pal_interaction_msgs
{

  class asrresult : public ros::Msg
  {
    public:
      char * text;
      uint8_t confidence;
      ros::Time start;
      ros::Time end;
      uint8_t tags_length;
      pal_interaction_msgs::actiontag st_tags;
      pal_interaction_msgs::actiontag * tags;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_text = strlen( (const char*) this->text);
      memcpy(outbuffer + offset, &length_text, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->text, length_text);
      offset += length_text;
      *(outbuffer + offset + 0) = (this->confidence >> (8 * 0)) & 0xFF;
      offset += sizeof(this->confidence);
      *(outbuffer + offset + 0) = (this->start.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->start.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->start.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->start.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->start.sec);
      *(outbuffer + offset + 0) = (this->start.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->start.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->start.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->start.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->start.nsec);
      *(outbuffer + offset + 0) = (this->end.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->end.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->end.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->end.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->end.sec);
      *(outbuffer + offset + 0) = (this->end.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->end.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->end.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->end.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->end.nsec);
      *(outbuffer + offset++) = tags_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < tags_length; i++){
      offset += this->tags[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_text;
      memcpy(&length_text, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_text; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_text-1]=0;
      this->text = (char *)(inbuffer + offset-1);
      offset += length_text;
      this->confidence =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->confidence);
      this->start.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->start.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->start.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->start.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->start.sec);
      this->start.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->start.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->start.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->start.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->start.nsec);
      this->end.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->end.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->end.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->end.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->end.sec);
      this->end.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->end.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->end.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->end.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->end.nsec);
      uint8_t tags_lengthT = *(inbuffer + offset++);
      if(tags_lengthT > tags_length)
        this->tags = (pal_interaction_msgs::actiontag*)realloc(this->tags, tags_lengthT * sizeof(pal_interaction_msgs::actiontag));
      offset += 3;
      tags_length = tags_lengthT;
      for( uint8_t i = 0; i < tags_length; i++){
      offset += this->st_tags.deserialize(inbuffer + offset);
        memcpy( &(this->tags[i]), &(this->st_tags), sizeof(pal_interaction_msgs::actiontag));
      }
     return offset;
    }

    const char * getType(){ return "pal_interaction_msgs/asrresult"; };
    const char * getMD5(){ return "f3597257ae9de52a0a19eb126d8461d7"; };

  };

}
#endif