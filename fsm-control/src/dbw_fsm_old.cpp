#include "ros/ros.h"
#include <iostream>
#include <string>
#include "fsm_control/servo_list.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
using namespace std;

// define states of car
typedef enum
{
    READY=1,
    MOVE,
    STOP
}tcar_state;

static tcar_state car_state = READY;


string data_cmd = "keep";

#define MAX_LEFT_STEER  460
#define CENTER_STEER 	333
#define MAX_RIGHT_STEER 260

#define THROTTLE_FORWARD_PWM  460      //#pwm value for max forward throttle
#define THROTTLE_REVERSE_PWM  270     // #pwm value for max reverse throttle
#define THROTTLE_STOPPED_PWM  370     // #pwm value for no movement

 // inputs data_cmd to change state

int throttle = 6;//THROTTLE_STOPPED_PWM;
int steer = 5;//CENTER_STEER;
const int v = 470; // const speed

//functions used by the periodic function
//############### ---- start section ---- ################
static void ready_func(void)
{
    //throttle = THROTTLE_STOPPED_PWM;
    //steer = CENTER_STEER;
    //fsm_control::servo_list servos_msg;
		//	servos_msg.steer_pulse = CENTER_STEER;
			//servos_msg.throttle_pulse = THROTTLE_STOPPED_PWM;
	//pub.publish(servos_msg);
    ROS_INFO("from READY");
    if (data_cmd == "keep")
        car_state = MOVE;

}

static void move_func(void)
{
    throttle = v;
    //fsm_control::servo_list servos_msg;
		//	servos_msg.steer_pulse = steer;
			//servos_msg.throttle_pulse = throttle;
	//pub.publish(servos_msg);
	 ROS_INFO("MOVE");
    if (data_cmd == "stop")
        car_state = STOP;
   

}

static void stop_func(void)
{
    //throttle = THROTTLE_REVERSE_PWM;
    //fsm_control::servo_list servos_msg;
			//servos_msg.steer_pulse = steer;
		//	servos_msg.throttle_pulse = THROTTLE_REVERSE_PWM;
	//pub.publish(servos_msg);
  //  cout<<throttle<<endl;
    //throttle = THROTTLE_STOPPED_PWM;
    //    servos_msg.throttle_pulse = THROTTLE_STOPPED_PWM;
    //pub.publish(servos_msg);
    //cout<<throttle.data<<endl;
    if (data_cmd == "keep")
        car_state = MOVE;
}

string get_state(void)
{ 
    string s;
    switch (car_state)
    {
        case READY:
            s= "READY";
        break;
        
        case MOVE:
            s= "MOVE";
        break;
        case STOP:
            s= "STOP";
        break;
    }
    return s;
}
//############### ---- end section ---- ################

//periodic function
void select_state_task(void)
{ cout<<"start "<<get_state()<<endl;
    switch(car_state)
    {
        case READY:
            ready_func();
            //cout<<"hi"<<car_state<<endl;
        break;

        case MOVE:
            move_func();
            //cout<<car_state<<endl;
        break;

        case STOP:
            stop_func();
            //cout<<car_state<<endl;
        break;
    }
    cout<<"end "<<get_state()<<endl;
}
//::ConstPtr&
 void planning_cb(const std_msgs::String msg)
{
     //data_cmd = msg.data;//->data.c_str(); 
    ROS_INFO("I heard: planning");//[%s], );//->data.c_str());
}
 
int main(int argc, char **argv)
{
        
       // init node with name trajectory
	ros::init(argc, argv, "dbw");

	ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("planning/cmd", 1000, planning_cb);
  	ros::Publisher pub = node.advertise<fsm_control::servo_list>("servo_cmd", 10);

    
	ros::Rate loop_rate(1);

    while (ros::ok())
	{	
	
        select_state_task();
        if (get_state() =="STOP")  //throttle = THROTTLE_REVERSE_PWM;
    	{
    	fsm_control::servo_list servos_msg_s;
			servos_msg_s.steer_pulse.data = steer;
			servos_msg_s.throttle_pulse.data = THROTTLE_REVERSE_PWM;
			pub.publish(servos_msg_s);
    		ROS_INFO("I heard: [%d]", throttle);
    		throttle = THROTTLE_STOPPED_PWM;
        	servos_msg_s.throttle_pulse.data= throttle;
    		pub.publish(servos_msg_s);
		}
		fsm_control::servo_list servos_msg;
		servos_msg.steer_pulse.data = steer;
		servos_msg.throttle_pulse.data = throttle;
		pub.publish(servos_msg);
		
		loop_rate.sleep();
		ROS_INFO("finished");
	}
	ROS_INFO("finished");
	ros::spin();


}

