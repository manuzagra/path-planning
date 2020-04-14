#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// struct Pose2D{
//   char type;
//   union {
//     struct {
//       double x;
//       double y;
//       double theta;
//     };
//     struct {
//       double s;
//       double d;
//     };
//   };
// };

struct Pose2D{
  double x;
  double y;
  double theta;
};



// car information
struct Car{
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
};

// detection information
struct Detection{
	unsigned int id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
};

// map map_waypoints
struct MapWaypoints{
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> s;
  std::vector<double> dx;
  std::vector<double> dy;
  std::vector<float> max_speed;
  std::vector<int> lanes_n;
  std::vector<float> lanes_width;

  // std::vector<double> operator[](std::size_t idx)
  // {
  //   return {x[idx], y[idx], s[idx], dx[idx], dy[idx], max_speed[idx], lanes_n[idx], lanes_width[idx]};
  // }
};



// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }
inline double mph2ms(double x) { return x * 0.44704; }
inline double ms2mph(double x) { return x * 2.23694; }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int closest_waypoint_index(double x, double y, const MapWaypoints &map) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < map.x.size(); ++i) {
    double map_x = map.x[i];
    double map_y = map.y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int next_waypoint_index(double x, double y, double theta, const MapWaypoints &map) {
  int closestWaypoint = closest_waypoint_index(x,y,map);
  double map_x = map.x[closestWaypoint];
  double map_y = map.y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == map.x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> cartesian_to_frenet(double x, double y, double theta, const MapWaypoints &map) {
  int next_wp = next_waypoint_index(x, y, theta, map);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = map.x.size()-1;
  }

  double n_x = map.x[next_wp]-map.x[prev_wp];
  double n_y = map.y[next_wp]-map.y[prev_wp];
  double x_x = x - map.x[prev_wp];
  double x_y = y - map.y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-map.x[prev_wp];
  double center_y = 2000-map.y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(map.x[i],map.y[i],map.x[i+1],map.y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> frenet_to_cartesian(double s, double d, const MapWaypoints &map) {
  int prev_wp = -1;

  while (s > map.s[prev_wp+1] && (prev_wp < (int)(map.s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%map.x.size();

  double heading = atan2((map.y[wp2]-map.y[prev_wp]),
                         (map.x[wp2]-map.x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-map.s[prev_wp]);

  double seg_x = map.x[prev_wp]+seg_s*cos(heading);
  double seg_y = map.y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

// // NOT TESTED, from knowledge
// // Return velocity along s and d in frenet coordinates
// std::vector<double> cartesian_to_frenetVelocity(double x, double y, const MapWaypoints &map)
// {
// int wp = closest_waypoint_index(x, y, map);
// double dx = map.dx[wp];
// double dy = map.dy[wp];
// double vd = vx*dx + vy*dy;
// double vs = -vx*dy + vy*dx;
//
// return {vs, vd};
//
// }


#endif  // HELPERS_H
