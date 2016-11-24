#ifndef _ROS_SERVICE_Empty_h
#define _ROS_SERVICE_Empty_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace roscpp
{

static const char EMPTY[] = "roscpp/Empty";

  class EmptyRequest : public ros::Msg
  {
    public:

    EmptyRequest()
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

    const char * getType(){ return EMPTY; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class EmptyResponse : public ros::Msg
  {
    public:

    EmptyResponse()
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

    const char * getType(){ return EMPTY; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class Empty {
    public:
    typedef EmptyRequest Request;
    typedef EmptyResponse Response;
  };

}
#endif
