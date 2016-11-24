#ifndef _ROS_nav_msgs_MapMetaData_h
#define _ROS_nav_msgs_MapMetaData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"
#include "geometry_msgs/Pose.h"

namespace nav_msgs
{

  class MapMetaData : public ros::Msg
  {
    public:
      ros::Time map_load_time;
      float resolution;
      uint32_t width;
      uint32_t height;
      geometry_msgs::Pose origin;

    MapMetaData():
      map_load_time(),
      resolution(0),
      width(0),
      height(0),
      origin()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->map_load_time.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->map_load_time.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->map_load_time.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->map_load_time.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->map_load_time.sec);
      *(outbuffer + offset + 0) = (this->map_load_time.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->map_load_time.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->map_load_time.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->map_load_time.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->map_load_time.nsec);
      union {
        float real;
        uint32_t base;
      } u_resolution;
      u_resolution.real = this->resolution;
      *(outbuffer + offset + 0) = (u_resolution.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_resolution.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_resolution.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_resolution.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->resolution);
      *(outbuffer + offset + 0) = (this->width >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->width >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->width >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->width >> (8 * 3)) & 0xFF;
      offset += sizeof(this->width);
      *(outbuffer + offset + 0) = (this->height >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->height >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->height >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->height >> (8 * 3)) & 0xFF;
      offset += sizeof(this->height);
      offset += this->origin.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->map_load_time.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->map_load_time.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->map_load_time.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->map_load_time.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->map_load_time.sec);
      this->map_load_time.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->map_load_time.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->map_load_time.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->map_load_time.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->map_load_time.nsec);
      union {
        float real;
        uint32_t base;
      } u_resolution;
      u_resolution.base = 0;
      u_resolution.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_resolution.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_resolution.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_resolution.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->resolution = u_resolution.real;
      offset += sizeof(this->resolution);
      this->width =  ((uint32_t) (*(inbuffer + offset)));
      this->width |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->width |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->width |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->width);
      this->height =  ((uint32_t) (*(inbuffer + offset)));
      this->height |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->height |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->height |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->height);
      offset += this->origin.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "nav_msgs/MapMetaData"; };
    const char * getMD5(){ return "10cfc8a2818024d3248802c00c95f11b"; };

  };

}
#endif