#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;

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
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  //////////////////////////////////////////////////
  // initial lane
  int lane = 1;
  // initial speed
  double vel = 0.0;
  //////////////////////////////////////////////////

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&vel]
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
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = deg2rad(j[1]["yaw"]);
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x_ = j[1]["previous_path_x"];
          auto previous_path_y_ = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s_ = j[1]["end_path_s"];
          double end_path_d_ = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          nlohmann::json msgJson;

          std::vector<double> next_x_vals;
          std::vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

           // constants
           float lane_width = 4;
           float max_speed = 49;
           float dt = 0.02;
           // params
           float distance_between_wp = 40;
           int n_wp = 5;
           int total_n_points = 50;
           // acceleration
           double acc = 1.2;
           // number of previous points to keep
           int n_prev_points = 20;



           unsigned int prev_size = previous_path_x_.size();


           // set the prev traj values to
           // we do this to continue with the last path

           std::vector<double> previous_path_x;
           std::vector<double> previous_path_y;
           int min = prev_size > n_prev_points ? n_prev_points : prev_size;
           for(int i=0; i<min; ++i){
             previous_path_x.push_back(previous_path_x_[i]);
             previous_path_y.push_back(previous_path_y_[i]);
           }
           prev_size = previous_path_x.size();
           if (prev_size > 0){
             vector<double> sd = getFrenet(*(previous_path_x.end()-1), *(previous_path_y.end()-1), car_yaw, map_waypoints_x, map_waypoints_y);
             car_s = sd[0];
           }




           /////////////////////  Sensor Fusion
           // all the lanes are empty till the moment we detect something
           std::vector<bool> empty_lanes({true,true,true});

           for (int i = 0; i < sensor_fusion.size(); ++i){
             // Review 2 - Lane number of vehicle
             float vehicle_d = sensor_fusion[i][6];

             int vehicle_lane = int(vehicle_d/4); // [0,1,2] * 4

             // get vehicle speed and s
             double vehicle_vx = sensor_fusion[i][3];
             double vehicle_vy = sensor_fusion[i][4];
             double vehicle_speed = sqrt(vehicle_vx*vehicle_vx + vehicle_vy*vehicle_vy);
             double vehicle_s = sensor_fusion[i][5];

             // predict the position of the car at the end of the prev trajectory
             vehicle_s += ((double) prev_size * dt * vehicle_speed);

             // car in the same line
             if (vehicle_lane == lane){
               // too close in the same line
               if (vehicle_s > car_s && abs(vehicle_s - car_s) < 20){
                 empty_lanes[lane] = false;
               }
             // different lane
             }else{
               if ((vehicle_s < car_s && abs(vehicle_s - car_s) < 10) || (vehicle_s > car_s && abs(vehicle_s - car_s) < 20)){
                 empty_lanes[vehicle_lane] = false;
               }
             }
           }

           /////////////////////  finite state machine???
           if (!empty_lanes[lane])
           {
             if(lane > 0 && empty_lanes[lane-1]){
               lane -= 1;
             }else if(lane < 2 && empty_lanes[lane+1]){
               lane += 1;
             }else{
               vel -= acc;
             }
           }else if(vel < max_speed){
             vel += acc;
           }






           //////////////////////// Trajectory
           std::vector<double> ptsx;
           std::vector<double> ptsy;
           double ref_x = car_x;
           double ref_y = car_y;
           double ref_yaw = car_yaw;

           if (prev_size < 2){
             double prev_car_x = car_x - cos(ref_yaw);
             double prev_car_y = car_y - sin(ref_yaw);

             ptsx.push_back(prev_car_x);
             ptsx.push_back(ref_x);
             ptsy.push_back(prev_car_y);
             ptsy.push_back(ref_y);
           }else{
             ref_x = previous_path_x[prev_size - 1];
             ref_y = previous_path_y[prev_size - 1];

             double prev_ref_x = previous_path_x[prev_size - 2];
             double prev_ref_y = previous_path_y[prev_size - 2];
             ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

             ptsx.push_back(prev_ref_x);
             ptsx.push_back(ref_x);
             ptsy.push_back(prev_ref_y);
             ptsy.push_back(ref_y);
           }

           // append n_wp new waypoints, equaly spaced
           for(int i=1; i<=n_wp; ++i){
             std::vector<double> new_wp = getXY(car_s + i*distance_between_wp, (0.5 + lane)*lane_width, map_waypoints_s, map_waypoints_x, map_waypoints_y);
             ptsx.push_back(new_wp[0]);
             ptsy.push_back(new_wp[1]);
           }

           // to car coordinates
           for (int i=0; i < ptsx.size(); ++i){
             double shift_x = ptsx[i] - ref_x;
             double shift_y = ptsy[i] - ref_y;

             ptsx[i] = (shift_x*cos(-ref_yaw)) - (shift_y*sin(-ref_yaw));
             ptsy[i] = (shift_x*sin(-ref_yaw)) + (shift_y*cos(-ref_yaw));
           }

           // spline
           tk::spline spline;
           spline.set_points(ptsx, ptsy);

           // use the prev points
           for (int i=0; i < previous_path_x.size(); ++i){
             next_x_vals.push_back(previous_path_x[i]);
             next_y_vals.push_back(previous_path_y[i]);
           }

           // calculate how we break up spline points so that we travel at our desired reference velocity
           double target_x = distance_between_wp;
           double target_y = spline(target_x);

           double target_distance = sqrt((target_x * target_x) + (target_y * target_y));

           double x_add_on = 0;
           double n_points_between_wp = target_distance/(dt * vel/2.24); // 2.24 -> mps to mph

           // Generate points to total 50 points
           for(int i=1; i <= total_n_points-previous_path_x.size(); ++i){

             double x_point = x_add_on + (distance_between_wp)/n_points_between_wp;
             double y_point = spline(x_point);

             x_add_on = x_point;

             double x_ref = x_point;
             double y_ref = y_point;

             // to map coordinates
             x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
             y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

             x_point += ref_x;
             y_point += ref_y;

             // save the points
             next_x_vals.push_back(x_point);
             next_y_vals.push_back(y_point);
           }

           /**
          #######################################################################
           **/


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

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
