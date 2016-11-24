#ifndef _ROS_sensor_msgs_RelativeHumidity_h
#define _ROS_sensor_msgs_RelativeHumidity_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace sensor_msgs
{

  class RelativeHumidity : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float relative_humidity;
      float variance;

    RelativeHumidity():
      header(),
      relative_humidity(0),
      variance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->relative_humidity);
      offset += serializeAvrFloat64(outbuffer + offset, this->variance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->relative_humidity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->variance));
     return offset;
    }

    const char * getType(){ return "sensor_msgs/RelativeHumidity"; };
    const char * getMD5(){ return "8730015b05955b7e992ce29a2678d90f"; };

  };

}
#endif