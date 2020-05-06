#ifndef MAP_H
#define MAP_H

#include <vector>
#include <fstream>
#include <sstream>
#include <ros/package.h>

using std::string;
using std::vector;

/*
  load map data file which contain road waypoints in this format (x,y,NumOfLanes)
*/

class Map
{
public:
  Map(string map_path);
  vector<double> &getWaypointX() { return map_waypoint_x; };
  vector<double> &getWaypointY() { return map_waypoint_y; };

private:
  int laneNum{2};
  float laneWidth{40}; // in cm
  vector<double> map_waypoint_x;
  vector<double> map_waypoint_y;
};

Map::Map(string map_path)
{
  map_path = ros::package::getPath("planning") + "/src/trajectory/" + map_path;
  std::ifstream in_map(map_path, std::ifstream::in);
  string line;
  double tempX, tempY;
  char temp;
  while (getline(in_map, line))
  {
    std::istringstream iss(line);
    iss >> tempX >> temp >> tempY;
    map_waypoint_x.push_back(tempX);
    map_waypoint_y.push_back(tempY);
  }
}

#endif