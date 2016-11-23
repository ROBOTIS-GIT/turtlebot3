#ifndef _ROS_SERVICE_RequestServiceInfo_h
#define _ROS_SERVICE_RequestServiceInfo_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rosserial_msgs
{

static const char REQUESTSERVICEINFO[] = "rosserial_msgs/RequestServiceInfo";

  class RequestServiceInfoRequest : public ros::Msg
  {
    public:
      const char* service;

    RequestServiceInfoRequest():
      service("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_service = strlen(this->service);
      memcpy(outbuffer + offset, &length_service, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->service, length_service);
      offset += length_service;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_service;
      memcpy(&length_service, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_service; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_service-1]=0;
      this->service = (char *)(inbuffer + offset-1);
      offset += length_service;
     return offset;
    }

    const char * getType(){ return REQUESTSERVICEINFO; };
    const char * getMD5(){ return "1cbcfa13b08f6d36710b9af8741e6112"; };

  };

  class RequestServiceInfoResponse : public ros::Msg
  {
    public:
      const char* service_md5;
      const char* request_md5;
      const char* response_md5;

    RequestServiceInfoResponse():
      service_md5(""),
      request_md5(""),
      response_md5("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_service_md5 = strlen(this->service_md5);
      memcpy(outbuffer + offset, &length_service_md5, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->service_md5, length_service_md5);
      offset += length_service_md5;
      uint32_t length_request_md5 = strlen(this->request_md5);
      memcpy(outbuffer + offset, &length_request_md5, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->request_md5, length_request_md5);
      offset += length_request_md5;
      uint32_t length_response_md5 = strlen(this->response_md5);
      memcpy(outbuffer + offset, &length_response_md5, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->response_md5, length_response_md5);
      offset += length_response_md5;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_service_md5;
      memcpy(&length_service_md5, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_service_md5; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_service_md5-1]=0;
      this->service_md5 = (char *)(inbuffer + offset-1);
      offset += length_service_md5;
      uint32_t length_request_md5;
      memcpy(&length_request_md5, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_request_md5; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_request_md5-1]=0;
      this->request_md5 = (char *)(inbuffer + offset-1);
      offset += length_request_md5;
      uint32_t length_response_md5;
      memcpy(&length_response_md5, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_response_md5; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_response_md5-1]=0;
      this->response_md5 = (char *)(inbuffer + offset-1);
      offset += length_response_md5;
     return offset;
    }

    const char * getType(){ return REQUESTSERVICEINFO; };
    const char * getMD5(){ return "c3d6dd25b909596479fbbc6559fa6874"; };

  };

  class RequestServiceInfo {
    public:
    typedef RequestServiceInfoRequest Request;
    typedef RequestServiceInfoResponse Response;
  };

}
#endif
