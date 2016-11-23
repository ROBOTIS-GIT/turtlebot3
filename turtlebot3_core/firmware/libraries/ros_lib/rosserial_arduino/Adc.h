#ifndef _ROS_rosserial_arduino_Adc_h
#define _ROS_rosserial_arduino_Adc_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rosserial_arduino
{

  class Adc : public ros::Msg
  {
    public:
      uint16_t adc0;
      uint16_t adc1;
      uint16_t adc2;
      uint16_t adc3;
      uint16_t adc4;
      uint16_t adc5;

    Adc():
      adc0(0),
      adc1(0),
      adc2(0),
      adc3(0),
      adc4(0),
      adc5(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->adc0 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->adc0 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->adc0);
      *(outbuffer + offset + 0) = (this->adc1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->adc1 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->adc1);
      *(outbuffer + offset + 0) = (this->adc2 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->adc2 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->adc2);
      *(outbuffer + offset + 0) = (this->adc3 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->adc3 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->adc3);
      *(outbuffer + offset + 0) = (this->adc4 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->adc4 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->adc4);
      *(outbuffer + offset + 0) = (this->adc5 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->adc5 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->adc5);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->adc0 =  ((uint16_t) (*(inbuffer + offset)));
      this->adc0 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->adc0);
      this->adc1 =  ((uint16_t) (*(inbuffer + offset)));
      this->adc1 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->adc1);
      this->adc2 =  ((uint16_t) (*(inbuffer + offset)));
      this->adc2 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->adc2);
      this->adc3 =  ((uint16_t) (*(inbuffer + offset)));
      this->adc3 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->adc3);
      this->adc4 =  ((uint16_t) (*(inbuffer + offset)));
      this->adc4 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->adc4);
      this->adc5 =  ((uint16_t) (*(inbuffer + offset)));
      this->adc5 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->adc5);
     return offset;
    }

    const char * getType(){ return "rosserial_arduino/Adc"; };
    const char * getMD5(){ return "6d7853a614e2e821319068311f2af25b"; };

  };

}
#endif