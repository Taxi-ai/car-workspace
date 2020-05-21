#include <iostream>

#include "ros/ros.h"
#include"sensor_msgs/Image.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using std::cout;

void listner(Mat &img)
{
  
}


int main(int argc, char **argv)
{

	// init node with name trajectory
	ros::init(argc, argv, "marks_node");

	ros::NodeHandle node;

	ros::Subscriber sub = n.subscribe("'sensors/camera_topic'", 1000, chatterCallback);

	ros::Rate loop_rate(10);