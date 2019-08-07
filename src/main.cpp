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

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;
using Eigen::MatrixXd;
using Eigen::VectorXd;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  int behavior_sm = 0; //state machine: 0: cruise, 1: follow/prepare lane change, 2: LC
  int curr_lane = 1; //current lane, initialize in lane 1
  double target_speed = 49.5;
  double spd_setpoint = 0.0; //desired velocity

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &behavior_sm,
               &curr_lane, &target_speed, &spd_setpoint]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];

          int prev_path_size = previous_path_x.size();
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //follow vehicle ahead
          if(prev_path_size > 0){
            car_s = end_path_s;
          }

          bool following_active = false;
          behavior_sm = 0;
          bool left_clear = true;
          bool right_clear = true;
          double tgt_follow_speed = target_speed;
          float d;
          double vx, vy, front_car_s, nearby_s;
          front_car_s = car_s + 50;

          for(int i = 0; i < sensor_fusion.size(); i++){
            d = sensor_fusion[i][6];
            nearby_s = sensor_fusion[i][5];
            if(d < (2+4*curr_lane+2) && d > (2+4*curr_lane-2)){
              vx = sensor_fusion[i][3];
              vy = sensor_fusion[i][4];
              tgt_follow_speed = sqrt(vx*vx + vy*vy);
              front_car_s = nearby_s;

              front_car_s += ((double)prev_path_size * 0.02 * tgt_follow_speed);
              if ((front_car_s > car_s) && (front_car_s-car_s < 30)){
                //spd_setpoint = tgt_follow_speed; //need to add PID control
                following_active = true;
                behavior_sm = 1;
                //check if there are cars on side, change curr_lane if not
                // if(curr_lane > 0){
                //   curr_lane -= 1;
                // }
              }

            } else if(d < (2 + 4*(curr_lane-1) + 2) && d > (2 + 4*(curr_lane-1) - 2) && curr_lane > 0 && nearby_s < front_car_s && nearby_s > car_s - 4) {
              left_clear = false;
            } else if(d < (2 + 4*(curr_lane+1) + 2) && d > (2 + 4*(curr_lane+1) - 2) && curr_lane < 2 && nearby_s < front_car_s && nearby_s > car_s - 4) {
              right_clear = false; //compute costs: keep in lane, lane left, lane right
            }

          }

          if(curr_lane == 0){
            left_clear = false;
          } else if (curr_lane == 2){
            right_clear = false;
          }
		  //cout << "check1" << endl;
          double increment;
          if(following_active){
            increment = 0.025*(tgt_follow_speed - spd_setpoint); //P controller
          } else {
            increment = 0.025*(target_speed - spd_setpoint); //resume cruise
          } //todo: consider distance error as well

          if(increment < 0.224){
            spd_setpoint += increment;
          } else {
            spd_setpoint += 0.224;
          }

          if(behavior_sm == 1){
            behavior_sm = 2;

            if (left_clear){
              curr_lane -= 1;
            } else if (right_clear){
              curr_lane += 1;
            }
            
          }

          //smooth lane keep
          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if (prev_path_size < 2){
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);

          } else {

            ref_x = previous_path_x[prev_path_size-1];
            ref_y = previous_path_y[prev_path_size-1];

            double ref_x_prev = previous_path_x[prev_path_size-2];
            double ref_y_prev = previous_path_y[prev_path_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

          }
		  //cout << "check2" << endl;
          vector<double> next_wp0 = getXY(car_s+30, 2+4*curr_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, 2+4*curr_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, 2+4*curr_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for(int i=0; i < ptsx.size(); i++){
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }
		  //cout << "check3" << endl;
          tk::spline s;

          s.set_points(ptsx, ptsy);
		  //cout << "check4" << endl;

          for(int i = 0; i < previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x*target_x)+(target_y*target_y));

          double x_add_on = 0;
		  //cout << "check5" << endl;
          for(int i = 0; i <= 50-previous_path_x.size(); i++){
            double N = (target_dist/(0.02*spd_setpoint/2.24));
            double x_point = x_add_on+target_x/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          /** Implementation notes:
          given car's localization data (s,d,speed)
          given nearby cars info (id,speed,s,d)

          //state machine: 0: cruise, 1: follow/prepare lane change, 2: LC
          */
          switch(behavior_sm){
            case 0:
              // cruise
              break;

            case 1:
              //prepare lane change
              break;

            case 2:
              //change lane
              break;
          }

          /*

          //cruise: just follow lane, at each timestep, create trajectories to
          30 meters ahead (lookahead dist)

          //FPLC: if car less than lookahead, use PID control to maintain car_speed
          also look for nearby cars < s_front, > s_ego - 3

          //LC: shift lanes into opening, go back to state 0


          filter nearby cars to only lookahead to 30 meters (?)

          find nearest cars

          built in loop that always maintains speed of car directly in front

          check if current speed == desired. if not, prepare lane change:
          check left and right lanes for nearby cars <= s val of front car and > current_s - 4.
          if none on either side, move to that lane.
          else, wait in lane
          */


          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

           //testing: simple lane keep
           // double dist_inc = 0.5;
           //
           // for(int i = 0; i < 50; i++){
           //   double next_s = car_s+(i+1)*dist_inc;
           //   double next_d = 6;
           //
           //   vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
           //
           //   next_x_vals.push_back(xy[0]);
           //   next_y_vals.push_back(xy[1]);
           // }



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
