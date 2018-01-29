#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <string.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

using namespace visualization_msgs;

class Turtlebot3MarkerServer
{
  public:
    Turtlebot3MarkerServer()
      : nh("~"), server("turtlebot3_marker_server")
    {
      std::string cmd_vel_topic;

      nh.param<std::string>("link_name", link_name, "/base_link");
      nh.param<double>("linear_scale", linear_scale, 1.0);
      nh.param<double>("angular_scale", angular_scale, 2.2);

      vel_pub = nh.advertise<geometry_msgs::Twist>("/interactive_marker_velocity_smoother/raw_cmd_vel", 1);
      createInteractiveMarkers();

      ROS_INFO("[turtlebot3_marker_server] Initialized.");
    }

    void processFeedback(
        const InteractiveMarkerFeedbackConstPtr &feedback );
  
  private:
    void createInteractiveMarkers();
  
    ros::NodeHandle nh;
    ros::Publisher vel_pub;
    interactive_markers::InteractiveMarkerServer server;
    
    double linear_scale;
    double angular_scale;
    
    std::string link_name;
};

void Turtlebot3MarkerServer::processFeedback(
    const InteractiveMarkerFeedbackConstPtr &feedback )
{
  // Handle angular change (yaw is the only direction in which you can rotate)
  double yaw = tf::getYaw(feedback->pose.orientation);
  
  geometry_msgs::Twist vel;
  vel.angular.z = angular_scale*yaw;
  vel.linear.x = linear_scale*feedback->pose.position.x;

  vel_pub.publish(vel);    
  
  // Make the marker snap back to turtlebot3
  server.setPose("turtlebot3_marker", geometry_msgs::Pose());
  
  server.applyChanges();
}

void Turtlebot3MarkerServer::createInteractiveMarkers()
{ 
  // create an interactive marker for our server
  InteractiveMarker int_marker;
  int_marker.header.frame_id = link_name;
  int_marker.name = "turtlebot3_marker";
  //int_marker.description = "Move the turtlebot3";
  
  InteractiveMarkerControl control;

  control.orientation_mode = InteractiveMarkerControl::FIXED;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  
  control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
  //control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  // Commented out for non-holonomic turtlebot3. If holonomic, can move in y.
  /*control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);*/
  
  server.insert(int_marker, boost::bind( &Turtlebot3MarkerServer::processFeedback, this, _1 ));
  
  server.applyChanges();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot3_marker_server");
  Turtlebot3MarkerServer turtleserver;
  
  ros::spin();
}
