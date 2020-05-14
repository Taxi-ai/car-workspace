#include <iostream>
#include <vector>
#include <math.h>

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
	float x = car.x, y = car.y;
	int WayPointIdx;
	vector<vector<double>> points;
	while (pointNum--)
	{
		WayPointIdx = NextWaypoint(x, y, car.yaw, path);
		if (WayPointIdx != -1)
			points.push_back({path[WayPointIdx].x + (20 + 40 * car.lane), path[WayPointIdx].y});
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

int main()
{
	Map map;
	Car car;
	// in current map data y -- > d and x -- > s where s && d is frenet corrdinat
	double goalX, goalY; // this point should be get from mobil app (I think using web API)
	int lane = 0, numPoints = 50;
	vector<WayPoint> path; // this path should come from web backend

	if (TEST)
	{
		goalX = 0;
		goalY = 100;

		car.speed = 1; // --- m/s
		for (double i = 0; i < goalY; i += 10)
		{
			path.push_back({0, i});
		}
	}

	//TODO:
	// * generat next 50 x,y points to move forward
	while (!ARRIVED)
	{
		vector<vector<double>> next_w;
		vector<double> ptsx, ptsy;

		next_w = genPoints(car, 3, path);
		for (auto p : next_w)
		{
			ptsx.push_back(p[0]);
			ptsy.push_back(p[1]);
		}

		//shift pts to make car the origin = make our calc easy
		shift(car, ptsx, ptsy);

		tk::spline s;
		s.set_points(ptsx, ptsy);
		vector<double> next_x_vals, next_y_vals;

		double target_x = 30;
		double target_y = s(target_x);
		double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

		double x_add_on = 0;

		//double dist_inc = 0.5;
		for (int i = 1; i <= numPoints; i++)
		{
			double N = (target_dist / (0.02 * 49.5 / 2.24));
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
		}
	}
}