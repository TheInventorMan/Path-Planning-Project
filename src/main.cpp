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

        int behavior_sm = 0;         // State machine variable: 0: cruise, 1: follow/prepare lane change, 2: LC
        int curr_lane = 1;           // Current lane, initialize in lane 1
        int target_lane = 1;         // Target lane to follow
        double target_speed = 49.0;  // Target speed during cruise
        double spd_setpoint = 0.0;   // Setpoint for throttle controller to follow

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
                                        // of the road.
                                        auto sensor_fusion = j[1]["sensor_fusion"];

                                        json msgJson;

                                        // Trajectory points to follow
                                        vector<double> next_x_vals;
                                        vector<double> next_y_vals;

                                        // Get current lane from current d coordinates
                                        if (car_d > 0 && car_d < 4) {
                                                curr_lane = 0;
                                        } else if (car_d > 4 && car_d < 8) {
                                                curr_lane = 1;
                                        } else if (car_d > 8 && car_d < 12) {
                                                curr_lane = 2;
                                        }

                                        behavior_sm = 0; // Reset state machine to cruise mode
                                        bool following_active = false; // Not currently following a vehicle ahead

                                        double tgt_follow_speed = target_speed; // Target following speed
                                        double front_car_s = 35; // Position of followed car
                                        double lowest_s = 10000; // S position of the nearest car in same lane
                                        int front_nearest = -1; // Index of nearest car in same lane

                                        // Find nearest car in the same lane
                                        for(int i = 0; i < sensor_fusion.size(); i++) {
                                                double front_d = sensor_fusion[i][6];
                                                double front_s = (double)sensor_fusion[i][5] - car_s;

                                                if(front_d > (4*curr_lane) && front_d < (4+4*curr_lane)) {
                                                        if ((front_s < lowest_s) && (front_s > 0)) {
                                                                front_nearest = i;
                                                                lowest_s = front_s;
                                                        }
                                                }
                                        }

                                        // Sanity check, change state machine, begin following if slower than ego vehicle
                                        if(front_nearest > -1) {
                                                double vx = sensor_fusion[front_nearest][3];
                                                double vy = sensor_fusion[front_nearest][4];
                                                tgt_follow_speed = sqrt(vx*vx + vy*vy);

                                                front_car_s = lowest_s;
                                                if (front_car_s < 30) {
                                                        following_active = true;
                                                        behavior_sm = 1;
                                                }
                                                else if (front_car_s > 35)
                                                {
                                                        following_active = false;
                                                        behavior_sm = 0;
                                                }
                                        }

                                        // Compute costs of lane changes
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

                                        for (int i = 0; i < sensor_fusion.size(); i++) {
                                                side_d = sensor_fusion[i][6];
                                                side_s = (double)sensor_fusion[i][5] - car_s;

                                                double side_vx = sensor_fusion[i][3];
                                                double side_vy = sensor_fusion[i][4];
                                                double rel_spd = sqrt(side_vx*side_vx + side_vy*side_vy) - car_speed; // Relative speed

                                                if ((side_d < (4 + 4*(curr_lane-1))) && (side_d > (4*(curr_lane-1)))) {
                                                        // Car on the left
                                                        if (side_s < nearest_left_s) { //unused for now
                                                                nearest_left_s = side_s;
                                                                nearest_left = i;
                                                        }
                                                        if (side_s < front_car_s && side_s > -10) {
                                                                num_left += 1; // Count cars potentially in the way of changing lanes

                                                                if (side_s < 6 && side_s > -6) { // Completely impermissible
                                                                        dummy_left_cost += 1000;
                                                                }

                                                                dummy_left_cost += 100/fabs(side_s); // Cost increases as car is closer to ego
                                                        }
                                                }

                                                if ((side_d < (4 + 4*(curr_lane+1))) && (side_d > (4*(curr_lane+1)))) {
                                                        // Car on the right
                                                        if (side_s < nearest_right_s) {
                                                                nearest_right_s = side_s;
                                                                nearest_right = i;
                                                        }
                                                        if (side_s < front_car_s && side_s > -10) {
                                                                num_right += 1; // Count cars potentially in the way of changing lanes

                                                                if (side_s < 6 && side_s > -6) {
                                                                        dummy_right_cost += 1000;
                                                                }
                                                                dummy_right_cost += 100/fabs(side_s);
                                                        }
                                                }

                                        }

                                        if(following_active) {
                                                 // Cost to remain in lane is proportional to current following speed
                                                keep_cost = 1-(tgt_follow_speed/target_speed);
                                        } else {
                                                // Cost is zero if no vehicle ahead
                                                keep_cost = 0.0;
                                        }

                                        // Compute costs bounded between 0 and 1
                                        left_cost = 1 - exp(-dummy_left_cost);
                                        right_cost = 1 - exp(-dummy_right_cost);

                                        // Cases for lanes on either side
                                        if(curr_lane == 0) {
                                                left_cost = 1;
                                        } else if (curr_lane == 2) {
                                                right_cost = 1;
                                        }

                                        //cout << "keep " << keep_cost << endl;
                                        //cout << "left " << left_cost << endl;
                                        //cout << "right " << right_cost << endl;

                                        // If commanded lane is not current localized lane, lane change is still in progress
                                        if (target_lane != curr_lane) {
                                                behavior_sm = 2;
                                        }

                                        if(behavior_sm == 1) { // Only change lane if in Prepare Lane Change mode
                                                if (left_cost < keep_cost && left_cost < right_cost) {
                                                        target_lane -= 1;
                                                        behavior_sm = 2;
                                                } else if (right_cost < keep_cost && right_cost < left_cost) {
                                                        target_lane += 1;
                                                        behavior_sm = 2;
                                                } else if (keep_cost > left_cost && keep_cost > right_cost) { // Break symmetry
                                                        if (curr_lane < 2) {
                                                                target_lane += 1;
                                                                behavior_sm = 2;
                                                        } else {
                                                                target_lane -= 1;
                                                                behavior_sm = 2;
                                                        }
                                                }
                                        }

                                        double increment;

                                        if(following_active) {
                                                increment = 0.035*(tgt_follow_speed - car_speed); // P controller for speed
                                        } else {
                                                increment = 0.035*(target_speed - car_speed); // Resume cruise when there is no car ahead
                                        }

                                        // Bound acceleration
                                        if(fabs(increment) < 0.224) {
                                                spd_setpoint += increment;
                                        } else if(increment > 0) {
                                                spd_setpoint += 0.224;
                                        } else {
                                                spd_setpoint -= 0.224;
                                        }

                                        // Compute next path from end of previous path
                                        if(prev_path_size > 0) {
                                                car_s = end_path_s;
                                        }

                                        // Compute coarse trajectory
                                        vector<double> ptsx;
                                        vector<double> ptsy;

                                        double ref_x = car_x;
                                        double ref_y = car_y;
                                        double ref_yaw = deg2rad(car_yaw);

                                        // Just use current position when starting out
                                        if (prev_path_size < 2) {
                                                double prev_car_x = car_x - cos(car_yaw);
                                                double prev_car_y = car_y - sin(car_yaw);

                                                ptsx.push_back(prev_car_x);
                                                ptsx.push_back(car_x);

                                                ptsy.push_back(prev_car_y);
                                                ptsy.push_back(car_y);

                                        } else {
                                                // Push back last points of previous trajectory for continuity
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

                                        // Get x,y coords of three lookahead waypoints at 30,60,90 meters ahead of car
                                        vector<double> next_wp0 = getXY(car_s+30, 2+4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                                        vector<double> next_wp1 = getXY(car_s+60, 2+4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                                        vector<double> next_wp2 = getXY(car_s+90, 2+4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

                                        // Add points to coarse trajectory
                                        ptsx.push_back(next_wp0[0]);
                                        ptsx.push_back(next_wp1[0]);
                                        ptsx.push_back(next_wp2[0]);

                                        ptsy.push_back(next_wp0[1]);
                                        ptsy.push_back(next_wp1[1]);
                                        ptsy.push_back(next_wp2[1]);

                                        // Rotate points 90 degrees to prevent vertical functions from being computed
                                        for(int i=0; i < ptsx.size(); i++) {
                                                double shift_x = ptsx[i]-ref_x;
                                                double shift_y = ptsy[i]-ref_y;

                                                ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
                                                ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
                                        }

                                        // Define spline using coarse trajectory
                                        tk::spline s;
                                        s.set_points(ptsx, ptsy);

                                        // Add remaining trajectory points on previous path to current trajectory for continuity
                                        for(int i = 0; i < previous_path_x.size(); i++) {
                                                next_x_vals.push_back(previous_path_x[i]);
                                                next_y_vals.push_back(previous_path_y[i]);
                                        }

                                        // Compute point spacing for smooth trajectory
                                        double target_x = 30.0;
                                        double target_y = s(target_x);
                                        double target_dist = sqrt((target_x*target_x)+(target_y*target_y));
                                        double x_add_on = 0;

                                        // Add smooth trajectory points onto current trajectory buffer
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
