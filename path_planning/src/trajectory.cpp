#include "trajectory.h"


TrajectoryGenerator::TrajectoryGenerator(MapWaypoints map, double dt):
  _map(map),
  _dt(dt)
{}

void TrajectoryGenerator::dt(double &t)
{
  _dt = t;
}

double TrajectoryGenerator::dt()
{
  return _dt;
}

void TrajectoryGenerator::map(MapWaypoints &m)
{
  _map = m;
}

MapWaypoints TrajectoryGenerator::map()
{
  return _map;
}
