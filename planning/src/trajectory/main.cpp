#include <iostream>
#include <vector>
#include <math.h>

#include "map.hpp"
#include "car.hpp"
#include "helper.hpp"
using std::cout;
using std::vector;

bool TEST = true, ARRIVED = false;
/*
Path that well return from path planning should be list of waypoint nodes as first node in the begin 
and goal nood at the end.
*/

vector<vector<float>> genPoints(Car &car, int pointNum, vector<WayPoint> &path)
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
	vector<vector<float>> points;
	while (pointNum--)
	{
		WayPointIdx = NextWaypoint(x, y, car.yaw, path);
		if (WayPointIdx != -1)
			points.push_back({path[WayPointIdx].x, path[WayPointIdx].y});
		else
			break;
	}
	// car finished path
	if(points.size()==0)
	{
		ARRIVED = true;
		return {{-1,-1}};
	}
	return points;
}
int main()
{
	Map map;
	Car car;
	// in current map data y -- > d and x -- > s where s && d is frenet corrdinat
	float goalX, goalY; // this point should be get from mobil app (I think using web API)
	int lane = 0;
	vector<WayPoint> path; // this path should come from web backend

	if (TEST)
	{
		goalX = 0;
		goalY = 100;

		car.speed = 1; // --- m/s
		vector<float> next_x, next_y;
		for (float i = 0; i < goalY; i += 10)
		{
			path.push_back({0, i});
		}
	}

	//TODO:
	// * generat next 50 x,y points to move forward
}