#include "ros/ros.h"
#include <iostream>
#include <string>
#include "fsm_control/servo_list.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <geometry_msgs/Twist.h>
using namespace std;

// define states of car
typedef enum
{
    READY=1,
    KEEP_LANE,
    STOP,
    LEFT_LANE,
    RIGHT_LANE,
    LEFT_TURN,
    RIGHT_TURN,
	KLEFT,
	KRIGHT
}tcar_state;

static tcar_state car_state = READY;


static string data_cmd ;

#define MAX_LEFT_STEER  455
#define CENTER_STEER 	360
#define MAX_RIGHT_STEER 265

#define THROTTLE_FORWARD_PWM  460      //#pwm value for max forward throttle
#define THROTTLE_REVERSE_PWM  270     // #pwm value for max reverse throttle
#define THROTTLE_STOPPED_PWM  370     // #pwm value for no movement

//turtle vars for simple test
float t_max_left=0.5;
float t_max_right= -0.5;
float t_center= 0.0;

 // inputs data_cmd to change state

static int throttle = THROTTLE_STOPPED_PWM;
static int steer = CENTER_STEER;
int v = 400;//470; // const speed

//functions used by the periodic function
//############### ---- start section ---- ################
static void ready_func(void)
{
    
    ROS_INFO("from READY");
    if (data_cmd == "keep")
        car_state = KEEP_LANE;

}

static void keep_lane_func(void)
{
    
	 ROS_INFO("From KEEP_LANE_Func");
    if (data_cmd == "stop")
        car_state = STOP;
	//else if (data_cmd == "kleft")
    //    car_state = KLEFT;
	//else if (data_cmd == "kright")
    //    car_state = KRIGHT;
	
	else if (data_cmd == "right_lane")
        car_state = RIGHT_LANE;
    else if (data_cmd == "left_lane")
        car_state = LEFT_LANE;
    else if (data_cmd == "turn_right")
        car_state = RIGHT_TURN;
    else if (data_cmd == "turn_left")
        car_state = LEFT_TURN;

}

static void stop_func(void)
{
    
    ROS_INFO("From STOP_FUNC");
    if (data_cmd == "keep")
        car_state = KEEP_LANE;
    else if (data_cmd == "right_lane")
        car_state = RIGHT_LANE;
    else if (data_cmd == "left_lane")
        car_state = LEFT_LANE;
    else if (data_cmd == "turn_right")
        car_state = RIGHT_TURN;
    else if (data_cmd == "turn_left")
        car_state = LEFT_TURN;
}

static void common_transition(void)
{
    //transition to main state : KEEP_LANE or STOP
    ROS_INFO("From common_transition");
    if (data_cmd == "keep")
        car_state = KEEP_LANE;
    else if (data_cmd == "stop")
    	car_state = STOP;
     
}



string get_state(void)
{ 
    string s;
    switch (car_state)
    {
        case READY:
            s= "READY";
        break;
        
        case KEEP_LANE:
            s= "KEEP_LANE";
        break;
        case STOP:
            s= "STOP";
        break;
        case LEFT_LANE:
            s= "LEFT_LANE";
        break;
        case RIGHT_LANE:
            s= "RIGHT_LANE";
        break;
        case LEFT_TURN:
            s= "LEFT_TURN";
        break;
        case RIGHT_TURN:
            s= "RIGHT_TURN";
        break;
    }
    return s;
}
//############### ---- end section ---- ################

//periodic function
void select_state_task(void)
{ cout<<"start transition from : "<<get_state()<<endl;
cout<<"hi : "<<data_cmd<<endl;
    switch(car_state)
    {
        case READY:
            cout<<"hi : READY : "<<car_state<<endl;
            ready_func();                
            //cout<<"hi"<<car_state<<endl;
        break;

        case KEEP_LANE:
            keep_lane_func();          
            //cout<<car_state<<endl;
        break;

        case STOP:
            stop_func();                 
            //cout<<car_state<<endl;
        break;
        
        case RIGHT_LANE:
            common_transition();          
            //cout<<car_state<<endl;
        break;
        
        case LEFT_LANE:
            common_transition();          
            //cout<<car_state<<endl;
        break;
        
        case RIGHT_TURN:
            common_transition();
            //cout<<car_state<<endl;
        break;
        
        case LEFT_TURN:
            common_transition();
            //cout<<car_state<<endl;
        break;
    }
    cout<<"current_state: "<<get_state()<<endl;
}


static void planning_cb(const std_msgs::String::ConstPtr& msg)
{
     data_cmd = msg->data.c_str(); 
     ROS_INFO("I heard: [%s]", msg->data.c_str());
     cout<<"data_cmd : 	"<<data_cmd<<endl;
     //select_state_task();
    
     
}
int rlane_c=1;
int llane_c=1;
int stop_c=1;
int main(int argc, char **argv)
{
        
       // init node 
	ros::init(argc, argv, "dbw_fsm");
	 

	ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("planning/cmd", 10000, planning_cb);
  	ros::Publisher pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  	ros::Rate loop_rate(10);
	 //fsm_control::servo_list servos_msg;
	int llane_c,lturn_c,rturn_c = 1; //some counters to publish sequence of commands during the same state

	while(ros::ok())
	{
		geometry_msgs::Twist msg;
      

		select_state_task();
		
		switch (car_state)
    	{
        	case READY:
            	throttle = THROTTLE_STOPPED_PWM;
    			steer = CENTER_STEER;
   				msg.angular.z = steer;//CENTER_STEER;
				msg.linear.x = throttle;//THROTTLE_STOPPED_PWM;
				pub.publish(msg);
        	break;
        
        	case KEEP_LANE:
            	 if (data_cmd == "kleft"){
					throttle = v;
   
				msg.angular.z  = steer+5;//t_center;//steer;
				msg.linear.x = throttle;//throttle;
				pub.publish(msg);}

				 if (data_cmd == "rleft"){
					throttle = v;
   
				msg.angular.z  = steer-5;//t_center;//steer;
				msg.linear.x = throttle;//throttle;
				pub.publish(msg);
				 }
	//else if (data_cmd == "kright")
    //    car_state = KRIGHT;
				else{
				throttle = v;
   
				msg.angular.z  = steer;//t_center;//steer;
				msg.linear.x = throttle;//throttle;
				pub.publish(msg);
				}
        	break;
        	
        	case STOP:
        	int static stop_c =1;
        	switch(stop_c){
        		case 1:
            	throttle = THROTTLE_REVERSE_PWM;
    			
				//msg.angular.z = steer;
				msg.linear.x = throttle;//THROTTLE_REVERSE_PWM;
				pub.publish(msg);
				++stop_c;
				ros::Duration(1).sleep();
  	  			break;
  	  			case 2:
  	  			//cout<<throttle<<endl;
    			throttle = THROTTLE_STOPPED_PWM;
    		    msg.linear.x = throttle;// THROTTLE_STOPPED_PWM;
    			pub.publish(msg);
    			stop_c=1;
    			ros::Duration(1.0).sleep();
			car_state = READY;
    			break;
    			}
        	break;
        	
        	case RIGHT_LANE:
        			static int rlane_c=1;
        			throttle = 1.0;
            	switch(rlane_c){
            		case 1:
            		steer = MAX_RIGHT_STEER;
            	msg.linear.x = throttle;
            	msg.angular.z = steer;//t_max_right;//MAX_RIGHT_STEER;
				pub.publish(msg);
				++rlane_c;
  	  			//cout<<throttle<<endl;
			 ros::Duration(5.0).sleep();
				
  	  			break;	
  	  			case  2:
  	  			msg.linear.x = throttle;
  	  			msg.angular.z =0.0;//t_center; //CENTER_STEER;
				pub.publish(msg);
				++rlane_c;
 ros::Duration(5.0).sleep();
				break;
				case 3:
				msg.linear.x = throttle;
				msg.angular.z = 0.5;//t_max_left;//MAX_LEFT_STEER;
				pub.publish(msg);
				++rlane_c;
 ros::Duration(5.0).sleep();
				break;
				case 4:
				msg.linear.x = throttle;
				msg.angular.z = 0.0;//t_center;//CENTER_STEER;
				pub.publish(msg);
				rlane_c =1;
 ros::Duration(5.0).sleep();
    			break;
    			//throttle = v;
    		    //msg.linear.x = 1.0;//throttle;
    			//pub.publish(msg);
    			}
        	break;
        
        	case LEFT_LANE:
            		static int llane_c=1;
            	switch(llane_c){
            		case 1:
            	msg.linear.x = 1.0;
            	msg.angular.z = 0.2;//t_max_right;//MAX_RIGHT_STEER;
				pub.publish(msg);
				++llane_c;
  	  			//cout<<throttle<<endl;
 ros::Duration(0.2).sleep();
  	  			break;
  	  			case  2:
  	  			msg.linear.x = 1.0;
  	  			msg.angular.z =0.0;//t_center; //CENTER_STEER;
				pub.publish(msg);
				++llane_c;
 ros::Duration(.5).sleep();
				break;
				case 3:
				msg.linear.x = 1.0;
				msg.angular.z =- 0.2;//t_max_left;//MAX_LEFT_STEER;
				pub.publish(msg);
				++llane_c;
 ros::Duration(0.2).sleep();
				break;
				case 4:
				msg.linear.x = 1.0;
				msg.angular.z = 0.0;//t_center;//CENTER_STEER;
				pub.publish(msg);
				llane_c =1;
 ros::Duration(.50).sleep();
				
    			break;
    			//throttle = v;
    		    //msg.linear.x = 1.0;//throttle;
    			//pub.publish(msg);
    			}
        	break;
        
        	case RIGHT_TURN:
            	int static tright_c =1;
        	switch(tright_c){
        		case 1:
            	throttle = v;
    			
				msg.angular.z = 0.2;
				msg.linear.x = throttle;//THROTTLE_REVERSE_PWM;
				pub.publish(msg);
				++tright_c;
				ros::Duration(1).sleep();
  	  			break;
  	  			case 2:
  	  			//cout<<throttle<<endl;
    			throttle = THROTTLE_STOPPED_PWM;
    		    msg.linear.x = throttle;// THROTTLE_STOPPED_PWM;
    			pub.publish(msg);
    			tright_c=1;
    			ros::Duration(1.0).sleep();
			//car_state = READY;
    			break;
    			}
        	break;
        
        	case LEFT_TURN:
            	int static tleft_c =1;
        	switch(tleft_c){
        		case 1:
            	throttle = v;
    			
				msg.angular.z = -0.2;
				msg.linear.x = throttle;//THROTTLE_REVERSE_PWM;
				pub.publish(msg);
				++tleft_c;
				ros::Duration(1).sleep();
  	  			break;
  	  			case 2:
  	  			//cout<<throttle<<endl;
    			throttle = THROTTLE_STOPPED_PWM;
    		    msg.linear.x = throttle;// THROTTLE_STOPPED_PWM;
    			pub.publish(msg);
    			tleft_c=1;
    			ros::Duration(1.0).sleep();
			car_state = READY;
    			break;
    			}
				
        	break;
    	}
		ROS_INFO("listening");
		
		ros::spinOnce();
		loop_rate.sleep();
		//++rlane_c;
		//++llane_c;
		//++lturn_c;
		//++rturn_c; 
	}

return 0;

}
