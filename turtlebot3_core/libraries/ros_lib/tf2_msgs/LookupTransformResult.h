#ifndef _ROS_tf2_msgs_LookupTransformResult_h
#define _ROS_tf2_msgs_LookupTransformResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_msgs/TF2Error.h"

namespace tf2_msgs
{

  class LookupTransformResult : public ros::Msg
  {
    public:
      geometry_msgs::TransformStamped transform;
      tf2_msgs::TF2Error error;

    LookupTransformResult():
      transform(),
      error()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->transform.serialize(outbuffer + offset);
      offset += this->error.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->transform.deserialize(inbuffer + offset);
      offset += this->error.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "tf2_msgs/LookupTransformResult"; };
    const char * getMD5(){ return "3fe5db6a19ca9cfb675418c5ad875c36"; };

  };

}
#endif