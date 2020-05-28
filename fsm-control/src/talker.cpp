#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("planning/cmd", 1000);
  ros::Rate loop_rate(0.3);

 
  int count = 1;
  while (ros::ok())
  {

    std_msgs::String msg;

    std::stringstream ss;
    

    switch (count)
    {
        case 1:
            ss<< "keep";
        break;
        
        case 2:
            ss<< "stop";
        break;
        case 3:
            ss<< "right_lane";
        break;
        case 4:
            ss<< "stop";
        break;
        case 5:
            ss<< "left_lane";
        break;
        case 6:
            ss<< "left_lane";
        break;
        case 7:
            ss<< "stop";
        break;
        case 8:
            ss<< "turn_left";
        break;
        case 9:
            ss<< "turn_left";
        break;
        case 10:
            ss<< "stop";
        break;
        case 11:
        	count = 0;
            ss<< "turn_right";
        break;
    }
  

	
    
    
    
   
 // ss << "stop";
 // ROS_INFO("%s \n", msg.data.c_str());
    msg.data = ss.str();
    ROS_INFO("%s \n", msg.data.c_str());
   chatter_pub.publish(msg);

    
    

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
// %EndTag(FULLTEXT)
