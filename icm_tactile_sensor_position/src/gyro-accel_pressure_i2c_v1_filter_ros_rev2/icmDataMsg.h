#ifndef _ROS_multimodal_tactile_sensor_icmDataMsg_h
#define _ROS_multimodal_tactile_sensor_icmDataMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace multimodal_tactile_sensor
{

  class icmDataMsg : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;

    icmDataMsg():
      name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
     return offset;
    }

    virtual const char * getType() override { return "multimodal_tactile_sensor/icmDataMsg"; };
    virtual const char * getMD5() override { return "c1f3d28f1b044c871e6eff2e9fc3c667"; };

  };

}
#endif
