#ifndef MAP_H
#define MAP_H

#include <vector>
#include <fstream>
#include <sstream>
// TODO : uncomment this affter finish
//#include <ros/package.h>

using std::string;
using std::vector;

/*
  load map data file which contain road waypoints in this format (x,y,NumOfLanes)
*/

struct WayPoint
{
  /* data */
  double x, y; // x -- > d && y -- > s
  int laneNum;
};

class Map
{
public:
  Map(string map_path = "mapData.csv");
  vector<WayPoint> getWaypoint() const { return map_waypoint_; };
  float getLaneWidth() { return laneWidth; }

private:
  float laneWidth{40}; // in cm
  vector<WayPoint> map_waypoint_;
};

Map::Map(string map_path)
{
  // TODO : uncomment this affter finish
  //map_path = ros::package::getPath("planning") + "/src/trajectory/" + map_path;
  std::ifstream in_map(map_path, std::ifstream::in);
  string line;
  double tempX, tempY;
  int laneNum;
  char temp;
  while (getline(in_map, line))
  {
    std::istringstream iss(line);
    iss >> tempX >> temp >> tempY >> temp >> laneNum;
    WayPoint p;
    p.x = tempX;
    p.y = tempY;
    p.laneNum = laneNum;
    map_waypoint_.push_back(p);
  }
}

#endif