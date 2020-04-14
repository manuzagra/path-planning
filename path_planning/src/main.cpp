#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
// #include "Eigen/Core"
// #include "Eigen/QR"
#include "helpers.h"
#include "json.hpp"

#include "trajectory.h"


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  MapWaypoints map_waypoints;

  // Waypoint map to read from
  std::string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  std::string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    double s;
    double d_x;
    double d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints.x.push_back(x);
    map_waypoints.y.push_back(y);
    map_waypoints.s.push_back(s);
    map_waypoints.dx.push_back(d_x);
    map_waypoints.dy.push_back(d_y);
    map_waypoints.max_speed.push_back(50); //mph2ms(50)); // all the waypoints have the same max speed
    map_waypoints.lanes_n.push_back(3);
    map_waypoints.lanes_width.push_back(4);
  }


  TrajectoryGenerator trajectory_generator(map_waypoints, 0.02);

  h.onMessage([&map_waypoints, &trajectory_generator]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = nlohmann::json::parse(s);

        std::string event = j[0].get<std::string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          Car car{j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"], deg2rad(j[1]["yaw"]), mph2ms(j[1]["speed"])};
          std::cout << "---------------------------------------------" << std::endl;
          std::cout << "---------------------------------------------" << std::endl;
          std::cout << "---------------------------------------------" << std::endl;
          std::cout << "car.x: " << car.x << std::endl;
          std::cout << "car.y: " << car.y << std::endl;
          std::cout << "car.s: " << car.s << std::endl;
          std::cout << "car.d: " << car.d << std::endl;
          std::cout << "car.yaw: " << car.yaw << std::endl;
          std::cout << "car.speed: " << car.speed << std::endl;

          // Previous path data given to the Planner
          // Only points that haven been executed
          Trajectory prev_traj;
          prev_traj.x = j[1]["previous_path_x"];
          prev_traj.y = j[1]["previous_path_y"];
          // auto previous_path_x = j[1]["previous_path_x"];
          // auto previous_path_y = j[1]["previous_path_y"];
          // // Previous path's end s and d values
          // double end_path_s = j[1]["end_path_s"];
          // double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          // [[ id, x, y, vx, vy, s, d]]
          std::vector<Detection> detections;
          for (int i=0; i<j[1]["sensor_fusion"].size(); ++i){
            detections.push_back(Detection{j[1]["sensor_fusion"][i][0],
                                           j[1]["sensor_fusion"][i][1],
                                           j[1]["sensor_fusion"][i][2],
                                           j[1]["sensor_fusion"][i][3],
                                           j[1]["sensor_fusion"][i][4],
                                           j[1]["sensor_fusion"][i][5],
                                           j[1]["sensor_fusion"][i][6]});
          }

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

           Trajectory traj = trajectory_generator.stay_in_line({car.x, car.y, car.yaw}, car.speed, detections, 100);

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          // std::cout << j << std::endl;
          nlohmann::json msgJson;
          msgJson["next_x"] = traj.x;
          msgJson["next_y"] = traj.y;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
