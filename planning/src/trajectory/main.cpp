#include <iostream>
#include <vector>
#include <math.h>

#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"

#include "map.hpp"
#include "car.hpp"
#include "helper.hpp"
#include "spline.h"

using std::cout;
using std::vector;

bool TEST = true, ARRIVED = false;
/*
Path that well return from path planning should be list of waypoint nodes as first node in the begin 
and goal nood at the end.
*/

vector<vector<double>> genPoints(Car &car, int pointNum, vector<WayPoint> &path)
{
	/*
		Generate (N = point num )points in the middle of current car lane using next N waypoint
		param:
			* car: reference to car object
			* pointNum: number of generated points
			* path: path that car should follow
		return:
			* vector of vector of points -- >{x,y}, of generated points
	*/
	double x = car.x, y = car.y;
	int WayPointIdx;
	vector<vector<double>> points;

	while (pointNum--)
	{
		WayPointIdx = NextWaypoint(x, y, car.yaw, path);
		if (WayPointIdx != -1)
		{
			points.push_back({path[WayPointIdx].x + (20 + 40 * car.lane), path[WayPointIdx].y});
			x = path[WayPointIdx].x;
			y = path[WayPointIdx].y;
		}

		else
			break;
	}

	// car finished path
	if (points.size() == 0)
	{
		ARRIVED = true;
		return {{-1, -1}};
	}

	return points;
}

void shift(Car &car, vector<double> &ptsx, vector<double> &ptsy)
{
	/*
		shift points givin in ptsx and ptsy using car x, y and yaw to make car as the origin.
	*/
	for (int i = 0; i < ptsx.size(); i++)
	{
		double shift_x = ptsx[i] - car.x;
		double shift_y = ptsy[i] - car.x;

		ptsx[i] = (shift_x * cos(0 - car.yaw) - shift_y * sin(0 - car.yaw));
		ptsy[i] = (shift_x * sin(0 - car.yaw) + shift_y * cos(0 - car.yaw));
	}
}

int main(int argc, char **argv)
{

	// init node with name trajectory
	ros::init(argc, argv, "planning_traj");

	ros::NodeHandle node;

	ros::Publisher pub = node.advertise<geometry_msgs::PoseArray>("final_waypoints", 2);

	ros::Rate loop_rate(10);

	Map map;
	Car car;
	// in current map data y -- > d and x -- > s where s && d is frenet corrdinat
	// TODO : accept goal point from back-end server
	double goalX, goalY; // this point should be get from mobil app (I think using web API)

	int lane = 0, numPoints = 10;
	vector<WayPoint> path; // this path should come from web backend

	if (TEST)
	{
		goalX = 20;
		goalY = 432;

		car.speed = 0.5; // --- m/s
		car.x = 20 + car.lane * 40;
		car.yaw = M_PI / 2;
		for (double i = 0; i < goalY; i += 10)
		{
			path.push_back({0, i});
		}
	}

	vector<double> prev_x, prev_y;
	while (ros::ok() && !ARRIVED)
	{
		vector<vector<double>> next_w;
		vector<double> ptsx, ptsy;
		int prev_path_size = prev_x.size();
		std::cout << "prev path size: " << prev_path_size << "\n";
		// we want new path to be tangent to current car yaw
		if (prev_path_size < 2)
		{
			double prev_car_x = car.x - cos(car.yaw);
			double prev_car_y = car.y - sin(car.yaw);
			ptsx.push_back(prev_car_x);
			ptsx.push_back(car.x);

			ptsy.push_back(prev_car_y);
			ptsy.push_back(car.y);
		}

		else
		{
			double ref_x = prev_x[prev_path_size - 1];
			double ref_y = prev_y[prev_path_size - 1];

			double ref_x_prev = prev_x[prev_path_size - 2];
			double ref_y_prev = prev_y[prev_path_size - 2];

			double ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

			ptsx.push_back(ref_x_prev);
			ptsx.push_back(ref_x);

			ptsy.push_back(ref_y_prev);
			ptsy.push_back(ref_y);
		}
		next_w = genPoints(car, 3, path);

		if (next_w.size() == 1) // path finished and genPoint send -1,-1
			continue;
		
		for (auto p : next_w)
		{
			ptsx.push_back(p[0]);
			ptsy.push_back(p[1]);
		}

		/*
		for(int i=0;i<ptsx.size();i++)
		{
			std::cout<<"x: "<<ptsx[i]<<"y: "<<ptsy[i]<<"\n"; 
		}
		*/

		//shift pts to make car the origin = make our calc easy
		shift(car, ptsx, ptsy);

		tk::spline s;
		s.set_points(ptsx, ptsy);
		vector<double> next_x_vals, next_y_vals;

		double target_x = 100;
		double target_y = s(target_x);
		double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

		// car speed in m/s and points in cm so /100 convert from m -> cm
		double N = (target_dist / (0.1 * car.speed * 100));

		double x_add_on = 0;

		for (int i = 1; i <= numPoints; i++)
		{
			double x_p = x_add_on + (target_x / N);
			double y_p = s(x_p);

			x_add_on = x_p;

			double x_ref = x_p;
			double y_ref = y_p;

			x_p = (x_ref * cos(car.yaw) - y_ref * sin(car.yaw));
			y_p = (x_ref * sin(car.yaw) + y_ref * cos(car.yaw));

			x_p += car.x;
			y_p += car.y;

			next_x_vals.push_back(x_p);
			next_y_vals.push_back(y_p);

			prev_x = next_x_vals;
			prev_y = next_y_vals;
		}
		int nex_waypoints_size = next_x_vals.size();
		// generate msg
		geometry_msgs::PoseArray msg;
		msg.header.stamp = ros::Time::now();
		for (int i = 0; i < nex_waypoints_size; i++)
		{
			//std::cout<<"x: "<<next_x_vals[i]
			// to make sure path will not pass goal point
			if (int(next_y_vals[i]) > goalY)
			{
				// to make sure that we reach exactly the goal with no error (here it was 5 cm error)
				geometry_msgs::Pose point;
				point.position.x = goalX;
				point.position.y = goalY;
				point.position.z = 0;
				ARRIVED = true;
				msg.poses.push_back(point);
				break;
			}
			geometry_msgs::Pose point;
			point.position.x = next_x_vals[i];
			point.position.y = next_y_vals[i];
			point.position.z = 0;

			msg.poses.push_back(point);
		}
		pub.publish(msg);

		// update car pos to be last trajectory point in last path
		if (TEST)
		{

			car.x = next_x_vals[nex_waypoints_size - 1];
			car.y = next_y_vals[nex_waypoints_size - 1];
		}
		loop_rate.sleep();
	}
	ROS_INFO("Path finished");
	ros::spin();
	// TODO
	/*
		* convert this to work when get path from web back-end or from any topic we sub to it
		* make it generate new next x and y with service as controller request when it finish last one
		* after finish reach goal, listen to new path and work again if it come.
	*/
}