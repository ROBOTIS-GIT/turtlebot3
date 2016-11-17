#ifndef _ROS_turtlebot3_msgs_DynamixelFeedback_h
#define _ROS_turtlebot3_msgs_DynamixelFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace turtlebot3_msgs
{

  class DynamixelFeedback : public ros::Msg
  {
    public:
      int32_t position1;
      int32_t position2;
      int32_t realtime_tick1;
      int32_t realtime_tick2;

    DynamixelFeedback():
      position1(0),
      position2(0),
      realtime_tick1(0),
      realtime_tick2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_position1;
      u_position1.real = this->position1;
      *(outbuffer + offset + 0) = (u_position1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position1);
      union {
        int32_t real;
        uint32_t base;
      } u_position2;
      u_position2.real = this->position2;
      *(outbuffer + offset + 0) = (u_position2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position2);
      union {
        int32_t real;
        uint32_t base;
      } u_realtime_tick1;
      u_realtime_tick1.real = this->realtime_tick1;
      *(outbuffer + offset + 0) = (u_realtime_tick1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_realtime_tick1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_realtime_tick1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_realtime_tick1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->realtime_tick1);
      union {
        int32_t real;
        uint32_t base;
      } u_realtime_tick2;
      u_realtime_tick2.real = this->realtime_tick2;
      *(outbuffer + offset + 0) = (u_realtime_tick2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_realtime_tick2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_realtime_tick2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_realtime_tick2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->realtime_tick2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_position1;
      u_position1.base = 0;
      u_position1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position1 = u_position1.real;
      offset += sizeof(this->position1);
      union {
        int32_t real;
        uint32_t base;
      } u_position2;
      u_position2.base = 0;
      u_position2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position2 = u_position2.real;
      offset += sizeof(this->position2);
      union {
        int32_t real;
        uint32_t base;
      } u_realtime_tick1;
      u_realtime_tick1.base = 0;
      u_realtime_tick1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_realtime_tick1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_realtime_tick1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_realtime_tick1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->realtime_tick1 = u_realtime_tick1.real;
      offset += sizeof(this->realtime_tick1);
      union {
        int32_t real;
        uint32_t base;
      } u_realtime_tick2;
      u_realtime_tick2.base = 0;
      u_realtime_tick2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_realtime_tick2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_realtime_tick2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_realtime_tick2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->realtime_tick2 = u_realtime_tick2.real;
      offset += sizeof(this->realtime_tick2);
     return offset;
    }

    const char * getType(){ return "turtlebot3_msgs/DynamixelFeedback"; };
    const char * getMD5(){ return "70a080ba72122dcbd0aa4ba4de407e58"; };

  };

}
#endif