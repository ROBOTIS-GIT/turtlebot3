#ifndef _ROS_nav_msgs_GetMapAction_h
#define _ROS_nav_msgs_GetMapAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "nav_msgs/GetMapActionGoal.h"
#include "nav_msgs/GetMapActionResult.h"
#include "nav_msgs/GetMapActionFeedback.h"

namespace nav_msgs
{

  class GetMapAction : public ros::Msg
  {
    public:
      nav_msgs::GetMapActionGoal action_goal;
      nav_msgs::GetMapActionResult action_result;
      nav_msgs::GetMapActionFeedback action_feedback;

    GetMapAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "nav_msgs/GetMapAction"; };
    const char * getMD5(){ return "e611ad23fbf237c031b7536416dc7cd7"; };

  };

}
#endif