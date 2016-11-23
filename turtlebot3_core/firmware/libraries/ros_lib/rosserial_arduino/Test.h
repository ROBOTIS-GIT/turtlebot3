#ifndef _ROS_SERVICE_Test_h
#define _ROS_SERVICE_Test_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rosserial_arduino
{

static const char TEST[] = "rosserial_arduino/Test";

  class TestRequest : public ros::Msg
  {
    public:
      const char* input;

    TestRequest():
      input("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_input = strlen(this->input);
      memcpy(outbuffer + offset, &length_input, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->input, length_input);
      offset += length_input;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_input;
      memcpy(&length_input, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_input; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_input-1]=0;
      this->input = (char *)(inbuffer + offset-1);
      offset += length_input;
     return offset;
    }

    const char * getType(){ return TEST; };
    const char * getMD5(){ return "39e92f1778057359c64c7b8a7d7b19de"; };

  };

  class TestResponse : public ros::Msg
  {
    public:
      const char* output;

    TestResponse():
      output("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_output = strlen(this->output);
      memcpy(outbuffer + offset, &length_output, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->output, length_output);
      offset += length_output;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_output;
      memcpy(&length_output, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_output; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_output-1]=0;
      this->output = (char *)(inbuffer + offset-1);
      offset += length_output;
     return offset;
    }

    const char * getType(){ return TEST; };
    const char * getMD5(){ return "0825d95fdfa2c8f4bbb4e9c74bccd3fd"; };

  };

  class Test {
    public:
    typedef TestRequest Request;
    typedef TestResponse Response;
  };

}
#endif
