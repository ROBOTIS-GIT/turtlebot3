#ifndef _ROS_geometry_msgs_TwistStamped_h
#define _ROS_geometry_msgs_TwistStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Twist.h"

namespace geometry_msgs
{

  class TwistStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::Twist twist;

    TwistStamped():
      header(),
      twist()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->twist.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->twist.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "geometry_msgs/TwistStamped"; };
    const char * getMD5(){ return "98d34b0043a2093cf9d9345ab6eef12e"; };

  };

}
#endif