#include "ros/ros.h"
#include <iostream>
#include <string>
#include "std_msgs/String.h"

using namespace std;
string lanes_cmd = "keep" , points_cmd = "keep";
bool in_keep_state = false;

void pointsCalback(const std_msgs::String::ConstPtr& msg)
{
  points_cmd = msg->data.c_str();
  //ROS_INFO("Points CMD heard: [%s]", points_cmd.c_str());
}

void lanesCalback(const std_msgs::String::ConstPtr& msg)
{
  lanes_cmd = msg->data.c_str();
  //ROS_INFO("Lanes CMD heard: [%s]", lanes_cmd.c_str());
}


int main(int argc,char **argv)
{
  ros::init(argc, argv, "behavior_planning");
  ros::NodeHandle node;
  ros::Subscriber points_sub = node.subscribe("perception/points_topic", 1, pointsCalback);
  ros::Subscriber lanes_sub = node.subscribe("perception/lanes_topic", 1, lanesCalback);
	ros::Publisher cmd_pub = node.advertise<std_msgs::String>("planning/cmd", 1);
	ros::Rate loop_rate(0.1);
  std_msgs::String msg;
  ROS_INFO("Planning node started");
  while(ros::ok())
  {
    if(points_cmd == "keep")
    {
      if(in_keep_state)
        {
          msg.data = lanes_cmd;
          cmd_pub.publish(msg);
        }
      else
      {
        in_keep_state = true;
        msg.data = "keep";
        cmd_pub.publish(msg);
        ros::Duration(0.1).sleep();
        msg.data = lanes_cmd;
        cmd_pub.publish(msg);
      }
    }
      
    else if (points_cmd != "ready")
    {
      msg.data = lanes_cmd;
      cmd_pub.publish(msg);
      in_keep_state = false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
