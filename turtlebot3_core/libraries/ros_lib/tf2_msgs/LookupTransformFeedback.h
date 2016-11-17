#ifndef _ROS_tf2_msgs_LookupTransformFeedback_h
#define _ROS_tf2_msgs_LookupTransformFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tf2_msgs
{

  class LookupTransformFeedback : public ros::Msg
  {
    public:

    LookupTransformFeedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return "tf2_msgs/LookupTransformFeedback"; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif