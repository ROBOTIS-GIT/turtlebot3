#ifndef _ROS_tf2_msgs_LookupTransformActionGoal_h
#define _ROS_tf2_msgs_LookupTransformActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "tf2_msgs/LookupTransformGoal.h"

namespace tf2_msgs
{

  class LookupTransformActionGoal : public ros::Msg
  {
    public:
      std_msgs::Header header;
      actionlib_msgs::GoalID goal_id;
      tf2_msgs::LookupTransformGoal goal;

    LookupTransformActionGoal():
      header(),
      goal_id(),
      goal()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->goal_id.serialize(outbuffer + offset);
      offset += this->goal.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->goal_id.deserialize(inbuffer + offset);
      offset += this->goal.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "tf2_msgs/LookupTransformActionGoal"; };
    const char * getMD5(){ return "f2e7bcdb75c847978d0351a13e699da5"; };

  };

}
#endif