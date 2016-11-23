#ifndef _ROS_turtlebot3_msgs_VersionInfo_h
#define _ROS_turtlebot3_msgs_VersionInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace turtlebot3_msgs
{

  class VersionInfo : public ros::Msg
  {
    public:
      const char* hardware;
      const char* firmware;
      const char* software;

    VersionInfo():
      hardware(""),
      firmware(""),
      software("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_hardware = strlen(this->hardware);
      memcpy(outbuffer + offset, &length_hardware, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->hardware, length_hardware);
      offset += length_hardware;
      uint32_t length_firmware = strlen(this->firmware);
      memcpy(outbuffer + offset, &length_firmware, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->firmware, length_firmware);
      offset += length_firmware;
      uint32_t length_software = strlen(this->software);
      memcpy(outbuffer + offset, &length_software, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->software, length_software);
      offset += length_software;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_hardware;
      memcpy(&length_hardware, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_hardware; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_hardware-1]=0;
      this->hardware = (char *)(inbuffer + offset-1);
      offset += length_hardware;
      uint32_t length_firmware;
      memcpy(&length_firmware, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_firmware; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_firmware-1]=0;
      this->firmware = (char *)(inbuffer + offset-1);
      offset += length_firmware;
      uint32_t length_software;
      memcpy(&length_software, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_software; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_software-1]=0;
      this->software = (char *)(inbuffer + offset-1);
      offset += length_software;
     return offset;
    }

    const char * getType(){ return "turtlebot3_msgs/VersionInfo"; };
    const char * getMD5(){ return "43e0361461af2970a33107409403ef3c"; };

  };

}
#endif