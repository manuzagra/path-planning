#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include <math.h>
#include <iostream>

#include "helpers.h"
#include "spline.h"
#include "cubic_trajectory.h"


struct Trajectory{
  std::vector<double> x;
  std::vector<double> y;
  double dt;
};

class TrajectoryGenerator{
public:
  TrajectoryGenerator(MapWaypoints map, double dt);
  virtual ~TrajectoryGenerator() = default;

  void dt(double &t);
  double dt();

  void map(MapWaypoints &m);
  MapWaypoints map();

  double space_between_points(double speed)
  {
    return speed * _dt;
  }

  std::vector<double> transform_point_to_map(std::vector<double> transform, std::vector<double> point)
  {
    // car -> map
    // transform = [x, y, theta]
    double new_x = transform[0] + point[0] * cos(transform[2]) - point[1] * sin(transform[2]);
    double new_y = transform[1] + point[0] * sin(transform[2]) + point[1] * cos(transform[2]);
    return {new_x, new_y};
  }

  std::vector<double> transform_point_from_map(std::vector<double> transform, std::vector<double> point)
  {
    // map -> car
    // transform = [x, y, theta]
    double new_x = (point[0]-transform[0]) * cos(-transform[2]) - (point[1]-transform[1]) * sin(-transform[2]);
    double new_y = (point[0]-transform[0]) * sin(-transform[2]) + (point[1]-transform[1]) * cos(-transform[2]);
    return {new_x, new_y};
  }

  double get_lane_centre_frenet(const Pose2D &p, int lane)
  {
    int next_wp = next_waypoint_index(p.x, p.y, p.theta, _map);
    std::vector<double> p_frenet = cartesian_to_frenet(p.x, p.y, p.theta, _map);
    // int(p_frenet[1]/_map.lanes_width[next_wp]) -> lane index
    // (lane_index + 0.5) * lane_width
    return (lane+0.5)*_map.lanes_width[next_wp];
  }








  Trajectory stay_in_line(const Trajectory &prev_traj, double initial_speed, int lane, const std::vector<Detection> &detections, float dist=100.)
  {
    // Previous trajectory will always have at least 2 points
    double theta = atan2(*(prev_traj.y.end()-1)-*(prev_traj.y.end()-2), *(prev_traj.x.end()-1)-*(prev_traj.x.end()-2));
    Pose2D start{*(prev_traj.x.end()-1), *(prev_traj.y.end()-1), theta};

    std::cout << "---------------------------------------------" << std::endl;
    std::cout << "start.x: " << start.x << std::endl;
    std::cout << "start.y: " << start.y << std::endl;
    std::cout << "start.theta: " << start.theta << std::endl;
    // store the waypoints to use in spline
    std::vector<double> wp_x;
    std::vector<double> wp_y;
    std::vector<double> wp_theta;
    std::vector<double> wp_speed;








    // int wp_index = next_waypoint_index(start.x, start.y, start.theta, _map) - 1;
    // float d = 0;
    // while(true){
    //   wp_x.push_back(_map.x[wp_index]);
    //   wp_y.push_back(_map.y[wp_index]);
    //   if (wp_index+1<_map.x.size()){
    //     wp_theta.push_back(atan2(_map.y[wp_index+1]-_map.y[wp_index], _map.x[wp_index+1]-_map.x[wp_index]));
    //   }else{
    //     wp_theta.push_back(atan2(_map.y[0]-_map.y[wp_index], _map.x[0]-_map.x[wp_index]));
    //   }
    //   wp_speed.push_back(_map.max_speed[wp_index]);
    //
    //   wp_index = wp_index+1<_map.x.size() ? wp_index+1 : 0;
    //   if (wp_x.size()>1){
    //     d += distance(*(wp_x.end()-2), *(wp_y.end()-2), *(wp_x.end()-1), *(wp_y.end()-1));
    //   }else if(wp_x.size()<1){
    //     d -= distance(start.x, start.y, *(wp_x.end()-1), *(wp_y.end()-1));
    //   }
    //   if (d>dist) break;
    // }







    // push the initial point
    // wp_x.push_back(start.x);
    // wp_y.push_back(start.y);
    // wp_theta.push_back(start.theta);
    // wp_speed.push_back(initial_speed);
    // get some waypoints more
    // it is easier in frenet
    std::vector<double> start_frenet = cartesian_to_frenet(start.x, start.y, start.theta, _map);
    // probably the car will already be in the middle of the line but just in case
    // calculate the desired d, the centre of the current lane
    double ref_d = get_lane_centre_frenet(start, lane);
    float distance_between_points = 20; // distance between waypoints
    int n_points = int(dist/distance_between_points) + 1; // number of waypoints to add
    n_points = n_points >= 2 ? n_points : 2;
    for (int i=0; i<n_points; ++i){
      std::vector<double> xy = frenet_to_cartesian(start_frenet[0]+i*distance_between_points, ref_d, _map);
      wp_x.push_back(xy[0]);
      wp_y.push_back(xy[1]);
      // calculate the position 1 meter before as well to get the yaw angle
      std::vector<double> xy_1 = frenet_to_cartesian(start_frenet[0]+i*distance_between_points-0.2, ref_d, _map);
      double theta = atan2(xy[1]-xy_1[1], xy[0]-xy_1[0]);
      wp_theta.push_back(theta);
      // get the velocity
      wp_speed.push_back(0.90*_map.max_speed[next_waypoint_index(xy[0], xy[1], theta, _map)]);

    }
    std::cout << "---------------------------------------------" << std::endl;
    std::cout << "wp_x.size(): " << wp_x.size() << std::endl;
    std::cout << "wp_y.size(): " << wp_y.size() << std::endl;
    std::cout << "wp_theta.size(): " << wp_theta.size() << std::endl;
    std::cout << "wp_speed.size(): " << wp_speed.size() << std::endl;
    // for (int i=0; i<wp_x.size(); ++i){
    //   std::cout << "i: " << i << std::endl;
    //   std::cout << "x: " << wp_x[i] << std::endl;
    //   std::cout << "y: " << wp_y[i] << std::endl;
    //   std::cout << "theta: " << wp_theta[i] << std::endl;
    //   std::cout << "speed: " << wp_speed[i] << std::endl;
    // }

    // Trajectory traj1(prev_traj);
    // traj1.x.insert(traj1.x.end(), wp_x.begin(), wp_x.end());
    // traj1.y.insert(traj1.y.end(), wp_y.begin(), wp_y.end());
    //
    // std::cout << "---------------------------------------------" << std::endl;
    // std::cout << "prev_traj.x.size(): " << prev_traj.x.size() << std::endl;
    // for(int i=0; i<traj1.x.size(); ++i){
    //   std::cout << "i: " << i << std::endl;
    //   std::cout << "traj.x[i]: " << traj1.x[i] << std::endl;
    //   std::cout << "traj.y[i]: " << traj1.y[i] << std::endl;
    // }
    //   std::cout << "start.x: " << start.x << std::endl;
    //   std::cout << "start.y: " << start.y << std::endl;
    //   std::cout << "start.theta: " << start.theta << std::endl;
    //
    // return traj1;







    // // now we have all the waypoint lets generate the points
    // // create a spline with
    // CubicTrajectory section;
    // Trajectory traj;//(prev_traj);
    // // traj.x.erase(traj.x.end()-1);
    // // traj.y.erase(traj.y.end()-1);
    // traj.dt = _dt;
    // for(int i=0; i<(wp_speed.size()-1); ++i){
    //     std::cout << "ms2mph((wp_speed[i] + wp_speed[i+1]) / 2): " << ms2mph((wp_speed[i] + wp_speed[i+1]) / 2) << std::endl;
    //   double distance_between_points = distance(wp_x[i], wp_y[i], wp_x[i+1], wp_y[i+1]);
    //   double t = distance_between_points / ((wp_speed[i] + wp_speed[i+1]) / 2);
    //   section.set_initial(wp_x[i], wp_y[i], wp_theta[i], wp_speed[i], 0);
    //   section.set_final(wp_x[i+1], wp_y[i+1], wp_theta[i+1], wp_speed[i+1], t);
    //   std::vector<std::vector<double> > xy = section.evaluate_complete(_dt);
    //   traj.x.insert(traj.x.end(), xy[0].begin(), xy[0].end());
    //   traj.y.insert(traj.y.end(), xy[1].begin(), xy[1].end());
    //
    //   // std::cout << "---------------------------------------------" << std::endl;
    //   // for(int i=0; i<xy[0].size(); ++i){
    //   //   std::cout << "i: " << i << std::endl;
    //   //   std::cout << "xy[0][i]: " << xy[0][i] << std::endl;
    //   //   std::cout << "xy[1][i]: " << xy[1][i] << std::endl;
    //   //
    //   // }
    //
    // }
    // std::cout << "---------------------------------------------" << std::endl;
    // std::cout << "prev_traj.x.size(): " << prev_traj.x.size() << std::endl;
    // std::cout << "---------------------------------------------" << std::endl;
    // for(int i=0; i<traj.x.size(); ++i){
    //   std::cout << "i: " << i << std::endl;
    //   std::cout << "traj.x[i]: " << traj.x[i] << std::endl;
    //   std::cout << "traj.y[i]: " << traj.y[i] << std::endl;
    //
    // }
    //
    // return traj;





    // we need to points at the beggining to ensure continuity
    wp_x.insert(wp_x.begin(), *(prev_traj.x.end()-2));
    wp_y.insert(wp_y.begin(), *(prev_traj.y.end()-2));

    std::vector<double> wp_x_car;
    std::vector<double> wp_y_car;
    for(int i=0; i <wp_x.size(); ++i){
      auto xy = transform_point_from_map({start.x, start.y, start.theta}, {wp_x[i], wp_y[i]});
      wp_x_car.push_back(xy[0]);
      wp_y_car.push_back(xy[1]);
    }

    tk::spline spline;
    spline.set_points(wp_x_car, wp_y_car);

    double target_x =  start.x + distance_between_points;
    double target_y = spline(target_x);
    double target_dist = distance(start.x, start.y, target_x, target_y);

    Trajectory traj(prev_traj);
    traj.x.erase(traj.x.end()-2, traj.x.end());
    traj.y.erase(traj.y.end()-2, traj.y.end());

    double x_start = *(traj.x.end()-1);
    for (int i=1; i<50; ++i){
      traj.x.push_back(x_start+i*0.4);
      traj.y.push_back(spline(x_start+i*0.4));
    }

    for(int i=0; i<traj.x.size(); ++i){
      std::cout << "i: " << i << std::endl;
      std::cout << "traj.x[i]: " << traj.x[i] << std::endl;
      std::cout << "traj.y[i]: " << traj.y[i] << std::endl;

    }

    return traj;






  }


private:
  double _dt;
  MapWaypoints _map;
  double _max_acc = 10;
};




#endif  // TRAJECTORY_H
