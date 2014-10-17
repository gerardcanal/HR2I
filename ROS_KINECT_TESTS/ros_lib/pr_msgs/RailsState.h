#ifndef _ROS_pr_msgs_RailsState_h
#define _ROS_pr_msgs_RailsState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr_msgs
{

  class RailsState : public ros::Msg
  {
    public:
      char * goal_station_name;
      double goal_station_distance;
      char * prev_station_name;
      double prev_station_distance;
      char * closest_station_name;
      double closest_station_distance;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_goal_station_name = strlen( (const char*) this->goal_station_name);
      memcpy(outbuffer + offset, &length_goal_station_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->goal_station_name, length_goal_station_name);
      offset += length_goal_station_name;
      union {
        double real;
        uint64_t base;
      } u_goal_station_distance;
      u_goal_station_distance.real = this->goal_station_distance;
      *(outbuffer + offset + 0) = (u_goal_station_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_goal_station_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_goal_station_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_goal_station_distance.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_goal_station_distance.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_goal_station_distance.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_goal_station_distance.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_goal_station_distance.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->goal_station_distance);
      uint32_t length_prev_station_name = strlen( (const char*) this->prev_station_name);
      memcpy(outbuffer + offset, &length_prev_station_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->prev_station_name, length_prev_station_name);
      offset += length_prev_station_name;
      union {
        double real;
        uint64_t base;
      } u_prev_station_distance;
      u_prev_station_distance.real = this->prev_station_distance;
      *(outbuffer + offset + 0) = (u_prev_station_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_prev_station_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_prev_station_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_prev_station_distance.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_prev_station_distance.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_prev_station_distance.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_prev_station_distance.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_prev_station_distance.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->prev_station_distance);
      uint32_t length_closest_station_name = strlen( (const char*) this->closest_station_name);
      memcpy(outbuffer + offset, &length_closest_station_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->closest_station_name, length_closest_station_name);
      offset += length_closest_station_name;
      union {
        double real;
        uint64_t base;
      } u_closest_station_distance;
      u_closest_station_distance.real = this->closest_station_distance;
      *(outbuffer + offset + 0) = (u_closest_station_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_closest_station_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_closest_station_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_closest_station_distance.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_closest_station_distance.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_closest_station_distance.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_closest_station_distance.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_closest_station_distance.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->closest_station_distance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_goal_station_name;
      memcpy(&length_goal_station_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_goal_station_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_goal_station_name-1]=0;
      this->goal_station_name = (char *)(inbuffer + offset-1);
      offset += length_goal_station_name;
      union {
        double real;
        uint64_t base;
      } u_goal_station_distance;
      u_goal_station_distance.base = 0;
      u_goal_station_distance.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_goal_station_distance.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_goal_station_distance.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_goal_station_distance.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_goal_station_distance.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_goal_station_distance.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_goal_station_distance.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_goal_station_distance.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->goal_station_distance = u_goal_station_distance.real;
      offset += sizeof(this->goal_station_distance);
      uint32_t length_prev_station_name;
      memcpy(&length_prev_station_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_prev_station_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_prev_station_name-1]=0;
      this->prev_station_name = (char *)(inbuffer + offset-1);
      offset += length_prev_station_name;
      union {
        double real;
        uint64_t base;
      } u_prev_station_distance;
      u_prev_station_distance.base = 0;
      u_prev_station_distance.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_prev_station_distance.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_prev_station_distance.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_prev_station_distance.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_prev_station_distance.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_prev_station_distance.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_prev_station_distance.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_prev_station_distance.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->prev_station_distance = u_prev_station_distance.real;
      offset += sizeof(this->prev_station_distance);
      uint32_t length_closest_station_name;
      memcpy(&length_closest_station_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_closest_station_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_closest_station_name-1]=0;
      this->closest_station_name = (char *)(inbuffer + offset-1);
      offset += length_closest_station_name;
      union {
        double real;
        uint64_t base;
      } u_closest_station_distance;
      u_closest_station_distance.base = 0;
      u_closest_station_distance.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_closest_station_distance.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_closest_station_distance.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_closest_station_distance.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_closest_station_distance.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_closest_station_distance.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_closest_station_distance.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_closest_station_distance.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->closest_station_distance = u_closest_station_distance.real;
      offset += sizeof(this->closest_station_distance);
     return offset;
    }

    const char * getType(){ return "pr_msgs/RailsState"; };
    const char * getMD5(){ return "ce17ce4ba137fd794491e233128e6635"; };

  };

}
#endif