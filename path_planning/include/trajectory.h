#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>

#include "helpers.h"


struct Trajectory{
  std::vector<double> x;
  std::vector<double> y;
  double dt;
};

class TrajectoryGenerator{
public:
  TrajectoryGenerator(double dt);
  virtual ~TrajectoryGenerator();

  stay_in_line


private:
  double dt;
};


#endif  // TRAJECTORY_H
