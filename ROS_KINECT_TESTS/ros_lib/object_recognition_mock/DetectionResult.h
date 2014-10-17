#ifndef _ROS_object_recognition_mock_DetectionResult_h
#define _ROS_object_recognition_mock_DetectionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/Image.h"
#include "object_recognition_mock/DetectedObject.h"

namespace object_recognition_mock
{

  class DetectionResult : public ros::Msg
  {
    public:
      sensor_msgs::Image Image;
      uint8_t ObjectNames_length;
      char* st_ObjectNames;
      char* * ObjectNames;
      uint8_t Detections_length;
      object_recognition_mock::DetectedObject st_Detections;
      object_recognition_mock::DetectedObject * Detections;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->Image.serialize(outbuffer + offset);
      *(outbuffer + offset++) = ObjectNames_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < ObjectNames_length; i++){
      uint32_t length_ObjectNamesi = strlen( (const char*) this->ObjectNames[i]);
      memcpy(outbuffer + offset, &length_ObjectNamesi, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->ObjectNames[i], length_ObjectNamesi);
      offset += length_ObjectNamesi;
      }
      *(outbuffer + offset++) = Detections_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < Detections_length; i++){
      offset += this->Detections[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->Image.deserialize(inbuffer + offset);
      uint8_t ObjectNames_lengthT = *(inbuffer + offset++);
      if(ObjectNames_lengthT > ObjectNames_length)
        this->ObjectNames = (char**)realloc(this->ObjectNames, ObjectNames_lengthT * sizeof(char*));
      offset += 3;
      ObjectNames_length = ObjectNames_lengthT;
      for( uint8_t i = 0; i < ObjectNames_length; i++){
      uint32_t length_st_ObjectNames;
      memcpy(&length_st_ObjectNames, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_ObjectNames; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_ObjectNames-1]=0;
      this->st_ObjectNames = (char *)(inbuffer + offset-1);
      offset += length_st_ObjectNames;
        memcpy( &(this->ObjectNames[i]), &(this->st_ObjectNames), sizeof(char*));
      }
      uint8_t Detections_lengthT = *(inbuffer + offset++);
      if(Detections_lengthT > Detections_length)
        this->Detections = (object_recognition_mock::DetectedObject*)realloc(this->Detections, Detections_lengthT * sizeof(object_recognition_mock::DetectedObject));
      offset += 3;
      Detections_length = Detections_lengthT;
      for( uint8_t i = 0; i < Detections_length; i++){
      offset += this->st_Detections.deserialize(inbuffer + offset);
        memcpy( &(this->Detections[i]), &(this->st_Detections), sizeof(object_recognition_mock::DetectedObject));
      }
     return offset;
    }

    const char * getType(){ return "object_recognition_mock/DetectionResult"; };
    const char * getMD5(){ return "ff5eaa0c7838288761622bd26a33514f"; };

  };

}
#endif