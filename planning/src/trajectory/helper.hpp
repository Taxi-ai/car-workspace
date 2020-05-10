#ifndef HELP_H
#define HELP_H

#include <vector>
#include <math.h>

#include "map.hpp"
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(float x, float y, const vector<WayPoint> &path)
{
	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (int i = 0; i < path.size(); ++i)
	{
		float map_x = path[i].x;
		float map_y = path[i].y;
		double dist = distance(x, y, map_x, map_y);
		if (dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(float x, float y, float theta, const vector<WayPoint> &path)
{
	int closestWaypoint = ClosestWaypoint(x, y, path);

	float map_x = path[closestWaypoint].x;
	float map_y = path[closestWaypoint].y;

	double heading = atan2((map_y - y), (map_x - x));

	double angle = fabs(theta - heading);
	angle = std::min(2 * pi() - angle, angle);

	if (angle > pi() / 2)
	{
		++closestWaypoint;
		if (closestWaypoint == path.size()) // TODO what we will do in this situation!!
		{
			// arrived to last waypint node
			closestWaypoint = -1;
		}
	}

	return closestWaypoint;
}

#endif