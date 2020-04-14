#ifndef CUBIC_PATH_H
#define CUBIC_PATH_H

// http://rossum.sourceforge.net/papers/CalculationsForRobotics/CubicPath.htm

#include <math.h>
#include <vector>

class CubicTrajectory{
public:
  CubicTrajectory() = default;
  ~CubicTrajectory() = default;

  void set_initial(double x, double y, double theta, double speed, double time)
  {
    x0 = x;
    y0 = y;
    theta0 = theta;
    speed0 = speed;
    t0 = time;
  }

  void set_final(double x, double y, double theta, double speed, double time)
  {
    x1 = x;
    y1 = y;
    theta1 = theta;
    speed1 = speed;
    t1 = time;
  }

  void calculate_coefficients()
  {
    double vx0 = speed0 * cos(theta0);
    double vy0 = speed0 * sin(theta0);
    double vx1 = speed1 * cos(theta1);
    double vy1 = speed1 * sin(theta1);
    ax = 6 * ((vx1+vx0)*(t1-t0)-2*(x1-x0)) / pow(t1-t0, 3.);
    bx = -2 * ((vx1+2*vx0)*(t1-t0)-3*(x1-x0)) / pow(t1-t0, 2.);
    cx = vx0;
    dx = x0;

    ay = 6 * ((vy1+vy0)*(t1-t0)-2*(y1-y0)) / pow(t1-t0, 3.);
    by = -2 * ((vy1+2*vy0)*(t1-t0)-3*(y1-y0)) / pow(t1-t0, 2.);
    cy = vy0;
    dy = y0;
  }

  std::vector<double> evaluate(double t)
  {
    double x = 1./6*ax*pow(t-t0, 3.) + 1./2*bx*pow(t-t0, 2.) + cx*(t-t0) + dx;
    double y = 1./6*ay*pow(t-t0, 3.) + 1./2*by*pow(t-t0, 2.) + cy*(t-t0) + dy;
    return {x, y};
  }

  std::vector<std::vector<double> > evaluate_complete(double dt)
  {
    calculate_coefficients();
    std::vector<double> x;
    std::vector<double> y;

    for(double t=t0; t<t1; t+=dt){
      std::vector<double> xy = evaluate(t);
      x.push_back(xy[0]);
      y.push_back(xy[1]);
    }
    return {x, y};
  }

private:
  void myfun(){};
  double x0, y0, theta0, speed0, t0;
  double x1, y1, theta1, speed1, t1;
  double ax, bx, cx, dx;
  double ay, by, cy, dy;
};



#endif // CUBIC_PATH_H
