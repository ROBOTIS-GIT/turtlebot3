#ifndef _ROS_tf2_msgs_LookupTransformAction_h
#define _ROS_tf2_msgs_LookupTransformAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "tf2_msgs/LookupTransformActionGoal.h"
#include "tf2_msgs/LookupTransformActionResult.h"
#include "tf2_msgs/LookupTransformActionFeedback.h"

namespace tf2_msgs
{

  class LookupTransformAction : public ros::Msg
  {
    public:
      tf2_msgs::LookupTransformActionGoal action_goal;
      tf2_msgs::LookupTransformActionResult action_result;
      tf2_msgs::LookupTransformActionFeedback action_feedback;

    LookupTransformAction():
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

    const char * getType(){ return "tf2_msgs/LookupTransformAction"; };
    const char * getMD5(){ return "7ee01ba91a56c2245c610992dbaa3c37"; };

  };

}
#endif