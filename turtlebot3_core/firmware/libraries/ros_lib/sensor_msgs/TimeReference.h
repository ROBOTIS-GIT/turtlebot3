#ifndef _ROS_sensor_msgs_TimeReference_h
#define _ROS_sensor_msgs_TimeReference_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ros/time.h"

namespace sensor_msgs
{

  class TimeReference : public ros::Msg
  {
    public:
      std_msgs::Header header;
      ros::Time time_ref;
      const char* source;

    TimeReference():
      header(),
      time_ref(),
      source("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->time_ref.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_ref.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time_ref.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time_ref.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_ref.sec);
      *(outbuffer + offset + 0) = (this->time_ref.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_ref.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time_ref.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time_ref.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_ref.nsec);
      uint32_t length_source = strlen(this->source);
      memcpy(outbuffer + offset, &length_source, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->source, length_source);
      offset += length_source;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->time_ref.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->time_ref.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time_ref.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time_ref.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time_ref.sec);
      this->time_ref.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->time_ref.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time_ref.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time_ref.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time_ref.nsec);
      uint32_t length_source;
      memcpy(&length_source, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_source; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_source-1]=0;
      this->source = (char *)(inbuffer + offset-1);
      offset += length_source;
     return offset;
    }

    const char * getType(){ return "sensor_msgs/TimeReference"; };
    const char * getMD5(){ return "fded64a0265108ba86c3d38fb11c0c16"; };

  };

}
#endif