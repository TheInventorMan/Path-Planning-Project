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
        int target_lane = 1;
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
                     &curr_lane, &target_lane, &target_speed, &spd_setpoint]
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
                                        //cout << sensor_fusion << endl;
                                        //follow vehicle ahead
                                        if(prev_path_size > 0) {
                                                car_s = end_path_s;
                                        }

                                        //get current lane
                                        if (car_d > 0 && car_d < 4) {
                                                curr_lane = 0;
                                        } else if (car_d > 4 && car_d < 8) {
                                                curr_lane = 1;
                                        } else if (car_d > 8 && car_d < 12) {
                                                curr_lane = 2;
                                        }

                                        behavior_sm = 0; //state machine
                                        bool following_active = false; //currently following vehicle ahead

                                        double tgt_follow_speed = target_speed; //target following speed
                                        double front_car_s = 35; //position of followed car
                                        double lowest_s = 10000; // nearest car in same lane, s position
                                        int front_nearest = -1; //index of nearest car in same lane

                                        for(int i = 0; i < sensor_fusion.size(); i++) { //find nearest car ahead
                                                double front_d = sensor_fusion[i][6];
                                                double front_s = (double)sensor_fusion[i][5] - car_s + 15;
                                                //cout << front_s << endl;
                                                if(front_d > (4*curr_lane) && front_d < (4+4*curr_lane)) {
                                                        if ((front_s < lowest_s) && (front_s > 0)) {
                                                                front_nearest = i;
                                                                lowest_s = front_s;
                                                        }
                                                }
                                        }

                                        if(front_nearest > -1) {
                                                double vx = sensor_fusion[front_nearest][3];
                                                double vy = sensor_fusion[front_nearest][4];
                                                tgt_follow_speed = sqrt(vx*vx + vy*vy);

                                                front_car_s = lowest_s;
                                                //front_car_s += ((double)prev_path_size * 0.02 * tgt_follow_speed);
                                                //cout << "check1" << endl;
                                                if (front_car_s < 30) {
                                                        following_active = true;
                                                        behavior_sm = 1;
                                                        //cout << "check2" << endl;
                                                } else if (front_car_s > 35) {
                                                        following_active = false;
                                                        behavior_sm = 0;
                                                }
                                        }

                                        double keep_cost, left_cost, right_cost;
                                        double nearest_left_s = 1000;
                                        double nearest_right_s = 1000;
                                        double side_d, side_s;
                                        int nearest_left = -1;
                                        int nearest_right = -1;
                                        int num_left = 0;
                                        int num_right = 0;

                                        double dummy_left_cost = 0.01;
                                        double dummy_right_cost = 0.01;
                                        //cout << "current_s " << car_s << endl;
                                        //if (behavior_sm == 1) { //in PLC mode, loop through sensor fusion data to find costs of lane changes
                                        cout << curr_lane << endl;

                                        for (int i = 0; i < sensor_fusion.size(); i++) {
                                                side_d = sensor_fusion[i][6];
                                                side_s = (double)sensor_fusion[i][5] - car_s + 15; //some weird offset (???)
                                                double side_vx = sensor_fusion[i][3];
                                                double side_vy = sensor_fusion[i][4];
                                                double rel_spd = sqrt(side_vx*side_vx + side_vy*vside_y) - car_speed;

                                                if ((side_d < (4 + 4*(curr_lane-1))) && (side_d > (4*(curr_lane-1)))) {
                                                        //car is on left
                                                        if (side_s < nearest_left_s) { //unused for now
                                                                nearest_left_s = side_s;
                                                                nearest_left = i;
                                                        }
                                                        if (side_s < front_car_s && side_s > -8) { //count cars in the way of changing lanes
                                                                num_left += 1;
                                                                cout << "car on left " << side_s << endl;
                                                        }
                                                        if (side_s < 0){
                                                          rel_spd *= -1;
                                                        }
                                                        dummy_left_cost += 1/fabs(side_s) - rel_spd;
                                                        //compute some cost variable value here

                                                }

                                                if ((side_d < (4 + 4*(curr_lane+1))) && (side_d > (4*(curr_lane+1)))) {
                                                        //car is on right
                                                        if (side_s < nearest_right_s) {
                                                                nearest_right_s = side_s;
                                                                nearest_right = i;
                                                        }
                                                        if (side_s < front_car_s && side_s > -8) { //count cars in the way of changing lanes
                                                                num_right += 1;
                                                                cout << "car on right " << side_s << endl;
                                                        } //todo: consider *approaching* vehicles (increase lookback, consider velocity)
                                                        //compute some cost variable here
                                                        if (side_s < 0){
                                                          rel_spd *= -1;
                                                        }
                                                        dummy_right_cost += 1/fabs(side_s) - rel_spd;
                                                }

                                        }

                                        //now compute costs
                                        if(following_active) {
                                                keep_cost = 1-(tgt_follow_speed/target_speed);
                                        } else {
                                                keep_cost = 0.0;
                                        }

                                        left_cost = 1 - exp(-dummy_left_cost)//2*num_left + 0.01;
                                        right_cost = 1 - exp(-dummy_right_cost)//2*num_right + 0.01; //todo

                                        if(curr_lane == 0) {
                                                left_cost = 1;
                                        } else if (curr_lane == 2) {
                                                right_cost = 1;
                                        }
                                        //}

                                        //cout << "keep " << keep_cost << endl;
                                        //cout << "left " << left_cost << endl;
                                        //cout << "right " << right_cost << endl;

                                        if (target_lane != curr_lane) {
                                                behavior_sm = 2;
                                        }

                                        if(behavior_sm == 1) {
                                                if (left_cost < keep_cost && left_cost < right_cost) {
                                                        target_lane -= 1;
                                                        behavior_sm = 2;
                                                } else if (right_cost < keep_cost && right_cost < left_cost) {
                                                        target_lane += 1;
                                                        behavior_sm = 2;
                                                }
                                        }

                                        double increment;

                                        if(following_active) {
                                                increment = 0.010*(tgt_follow_speed - car_speed); //P controller
                                        } else {
                                                increment = 0.025*(target_speed - car_speed); //resume cruise
                                        }

                                        if(fabs(increment) < 0.224) {
                                                spd_setpoint += increment;
                                        } else if(increment > 0) {
                                                spd_setpoint += 0.224;
                                        } else {
                                                spd_setpoint -= 0.224;
                                        }

                                        //smooth lane
                                        vector<double> ptsx;
                                        vector<double> ptsy;

                                        double ref_x = car_x;
                                        double ref_y = car_y;
                                        double ref_yaw = deg2rad(car_yaw);

                                        if (prev_path_size < 2) {
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

                                        vector<double> next_wp0 = getXY(car_s+30, 2+4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                                        vector<double> next_wp1 = getXY(car_s+60, 2+4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                                        vector<double> next_wp2 = getXY(car_s+90, 2+4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

                                        ptsx.push_back(next_wp0[0]);
                                        ptsx.push_back(next_wp1[0]);
                                        ptsx.push_back(next_wp2[0]);

                                        ptsy.push_back(next_wp0[1]);
                                        ptsy.push_back(next_wp1[1]);
                                        ptsy.push_back(next_wp2[1]);

                                        for(int i=0; i < ptsx.size(); i++) {
                                                double shift_x = ptsx[i]-ref_x;
                                                double shift_y = ptsy[i]-ref_y;

                                                ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
                                                ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
                                        }

                                        tk::spline s;
                                        s.set_points(ptsx, ptsy);

                                        for(int i = 0; i < previous_path_x.size(); i++) {
                                                next_x_vals.push_back(previous_path_x[i]);
                                                next_y_vals.push_back(previous_path_y[i]);
                                        }

                                        double target_x = 30.0;
                                        double target_y = s(target_x);
                                        double target_dist = sqrt((target_x*target_x)+(target_y*target_y));

                                        double x_add_on = 0;

                                        for(int i = 0; i <= 50-previous_path_x.size(); i++) {
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

                                           /*

                                           //cruise: just follow lane, at each timestep, create trajectories to
                                           30 meters ahead (lookahead dist)

                                           //FPLC: if car less than lookahead, use PID control to maintain car_speed
                                           also look for nearby cars < s_front, > s_ego - 7

                                           //LC: shift lanes into opening, go back to state 0


                                           filter nearby cars to only lookahead to 30 meters (?)

                                           find nearest cars

                                           built in loop that always maintains speed of car directly in front

                                           check if current speed == desired. if not, prepare lane change:
                                           check left and right lanes for nearby cars <= s val of front car and > current_s - 4.
                                           if none on either side, move to that lane.
                                           else, wait in lane
                                         */


                                        msgJson["next_x"] = next_x_vals;
                                        msgJson["next_y"] = next_y_vals;

                                        auto msg = "42[\"control\","+ msgJson.dump()+"]";

                                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                                } // end "telemetry" if
                        } else {
                                // Manual driving
                                std::string msg = "42[\"manual\",{}]";
                                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                        }
                } // end websocket if
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
