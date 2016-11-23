#ifndef _ROS_geometry_msgs_AccelWithCovarianceStamped_h
#define _ROS_geometry_msgs_AccelWithCovarianceStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/AccelWithCovariance.h"

namespace geometry_msgs
{

  class AccelWithCovarianceStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::AccelWithCovariance accel;

    AccelWithCovarianceStamped():
      header(),
      accel()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->accel.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->accel.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "geometry_msgs/AccelWithCovarianceStamped"; };
    const char * getMD5(){ return "96adb295225031ec8d57fb4251b0a886"; };

  };

}
#endif