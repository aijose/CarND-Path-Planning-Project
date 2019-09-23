#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
//#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"

#define TRAJECTORY_POINTS 50
#define OVERLAP_POINTS 2
#define DT 0.02
#define MPH_TO_MS 2.24

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

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

  Vehicle ego;
  ego.lane = 1;
  ego.state = "KL";
  ego.map_waypoints_x = map_waypoints_x;
  ego.map_waypoints_y = map_waypoints_y;
  ego.map_waypoints_s = map_waypoints_s;

  h.onMessage([&ego, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          int prev_size = previous_path_x.size();

          //if (prev_size > 0) {
          //  car_s = end_path_s;
          //}

          // find ref_v to use

          ego.x = car_x;
          ego.y = car_y;
          ego.s = car_s;
          ego.d = car_d;
          ego.determine_lane(car_d);
          ego.yaw = car_yaw;
          ego.previous_path_x.resize(previous_path_x.size());
          ego.previous_path_y.resize(previous_path_y.size());
          for(int i=0; i < previous_path_x.size(); i++) {
              ego.previous_path_x[i] = previous_path_x[i];
              ego.previous_path_y[i] = previous_path_y[i];
          }

          double min_cost = 1000000.0;
          Trajectory trajectory, best_trajectory;
          string best_state;
          vector<std::string> states = ego.successor_states();
          for(int i=0; i < states.size(); i++) {
              std::string state = states[i];
              trajectory = ego.generate_trajectory(state, sensor_fusion);
              float cost = ego.compute_cost(trajectory);
              if (cost < min_cost) {
                  min_cost = cost;
                  best_trajectory = trajectory;
                  best_state = state;
              }
          }

          ego.previous_trajectory = best_trajectory;
          ego.state = best_state;
          next_x_vals = best_trajectory.xlocs;
          next_y_vals = best_trajectory.ylocs;

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

// Initializes Vehicle
Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, float s, float v, float a, string state) {
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  this->state = state;
  max_acceleration = -1;
}

Vehicle::~Vehicle() {}

Trajectory Vehicle::generate_trajectory(string state, vector<vector<double>> sensor_fusion) {
  // Given a possible next state, generate the appropriate trajectory to realize
  //   the next state.
    Trajectory trajectory;
    if (state.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(sensor_fusion);
        trajectory.intended_lane = lane;
    } 
    else if(state.compare("LCL") == 0) {
        trajectory = lane_change_trajectory(lane-1, sensor_fusion);
        trajectory.intended_lane = lane-1;
    }
    else {
        trajectory = lane_change_trajectory(lane+1, sensor_fusion);
        trajectory.intended_lane = lane+1;
    }

  return trajectory;
}

bool Vehicle::is_lane_change_safe(int current_lane, int intended_lane, float ego_s, vector<vector<double>> sensor_fusion) {
    double front_margin = 35.0;
    double rear_margin = 35.0;

    if(current_lane == 1) {
        front_margin = 40.0;
    }
    Vehicle vehicle_ahead, vehicle_behind;
    get_nearest_vehicles(ego_s, sensor_fusion, intended_lane, vehicle_ahead, vehicle_behind);
    double speed_buffer = std::max(0.0f,vehicle_behind.v - v)*(40.0/v);
    if(vehicle_ahead.s - ego_s > front_margin && ego_s - vehicle_behind.s > rear_margin + speed_buffer)
        return true;
    else
        return false;
}


Trajectory Vehicle::constant_speed_trajectory() {
  // Generate a constant speed trajectory.
  Trajectory trajectory;
  return trajectory;
}


vector<string> Vehicle::successor_states() {
  // Provides the possible next states given the current state for the FSM 
  vector<string> states;
  states.push_back("KL");
  string state = this->state;
  if(state.compare("KL") == 0) {
    states.push_back("LCL");
    states.push_back("LCR");
  }     
  if(state.compare("LCL") == 0) {
    states.push_back("LCL");
  }
  if(state.compare("LCR") == 0) {
    states.push_back("LCR");
  }
  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}

Trajectory Vehicle::keep_lane_trajectory(vector<vector<double>> sensor_fusion) {
  // Generate a keep lane trajectory.
  Trajectory trajectory;
  int prev_size = previous_path_x.size();
  int intended_lane;
  bool too_close = false;
  bool very_close = false;
  double car_x = x;
  double car_y = y;
  double car_s = s;
  double car_yaw = yaw;
  Vehicle vehicle_ahead, vehicle_behind;
  int overlap_points = std::min(OVERLAP_POINTS,prev_size);
  
  int consumed_points = previous_trajectory.xlocs.size() - previous_path_x.size();
  if(prev_size != 0) {
      v = previous_trajectory.velocities[consumed_points-1];
  }
  else {
      v = 0.0;
  }

  double check_car_s, spacing;
  intended_lane = lane;

  get_nearest_vehicles(car_s, sensor_fusion, lane, vehicle_ahead, vehicle_behind);
  if(vehicle_ahead.s < 100000) {
      double check_speed = vehicle_ahead.v;
      //check_car_s = vehicle_ahead.s + ((double) overlap_points*DT*check_speed);
      check_car_s = vehicle_ahead.s;
      spacing = check_car_s - car_s;
      
      if (spacing <= 30 && spacing > 10) {
          too_close = true;
      } else if (spacing <= 10) {
          very_close = true;
      }
  }
  
  if (too_close && v > vehicle_ahead.v) {
      v -= 0.224*(1.0 - spacing/30.0); // Decrement by about 5 m/s
  }
  else if (very_close) {
      v -= 0.224*(1.0 - 1.0/3.0) + 0.4*(1.0-spacing/10.0);
  }
  else if (v < 49.0) {
      v += 0.112;
  }
  
  // Create a list of widely spaced (x,y) points, evenly spaced at 30 m
  // Later we will interpolate these waypoints with a spline and fill it in
  // with more points that control
  
  vector<double> ptsx;
  vector<double> ptsy;
  
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);
  
  if (prev_size < 2) {
    double prev_car_x = car_x - v*DT*cos(ref_yaw);
    double prev_car_y = car_y - v*DT*sin(ref_yaw);
  
    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);
  
    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  
  }
  else {
    ref_x = previous_path_x[overlap_points-1];
    ref_y = previous_path_y[overlap_points-1];
  
    double ref_x_prev = previous_path_x[overlap_points - 2];
    double ref_y_prev = previous_path_y[overlap_points - 2];
  
    ref_yaw =  atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
  
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }
  
  vector<double> next_wp0 = getXY(car_s+30, (2+4*intended_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s+60, (2+4*intended_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s+90, (2+4*intended_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  
  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);
  
  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
  
  for (int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
  
    ptsx[i] = (shift_x * cos(0 -  ref_yaw) - shift_y*sin(0-ref_yaw));
    ptsy[i] = (shift_x * sin(0 -  ref_yaw) + shift_y*cos(0-ref_yaw));
  }
  
  tk::spline s;
  
  s.set_points(ptsx, ptsy);
  
  for (int i = 0; i < overlap_points; i++) {
    trajectory.xlocs.push_back(previous_path_x[i]);
    trajectory.ylocs.push_back(previous_path_y[i]);
    trajectory.velocities.push_back(previous_trajectory.velocities[consumed_points+i]);
  }
  
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_distance = sqrt(target_x*target_x + target_y*target_y);
  
  double x_add_on = 0.0;
  
  for (int i = 0; i <= TRAJECTORY_POINTS-overlap_points; i++) {
    double x_point  = x_add_on + (target_x/target_distance)*(DT*v/MPH_TO_MS);
    double y_point  = s(x_point);

    if(too_close && v > vehicle_ahead.v) {
        v -= 0.112;
    }
    else if (very_close) {
        v -= 0.224*(1.0 - 1.0/3.0) + 0.4*(1.0-spacing/10.0);
    }
    else if(v < 49.0) {
        v += 0.112;
    }
  
    x_add_on = x_point;
  
    double x_ref = x_point;
    double y_ref = y_point;
  
    x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));
  
    x_point += ref_x;
    y_point += ref_y;
  
    trajectory.xlocs.push_back(x_point);
    trajectory.ylocs.push_back(y_point);
    trajectory.velocities.push_back(v);
  }

  get_nearest_vehicles(car_s, sensor_fusion, intended_lane, vehicle_ahead, vehicle_behind);
  trajectory.lane_speed = vehicle_ahead.v;
  trajectory.vehicle_ahead_distance = vehicle_ahead.s - car_s;
  trajectory.intended_lane = intended_lane;
  
  return trajectory;
}

Trajectory Vehicle::lane_change_trajectory(int intended_lane, vector<vector<double>> sensor_fusion) {
  Trajectory trajectory;
  int prev_size = previous_path_x.size();
  bool too_close = false;
  double car_x = x;
  double car_y = y;
  double car_s = s;
  double car_yaw = yaw;
  Vehicle vehicle_ahead, vehicle_behind;
  //double v = this->v;
  int overlap_points = std::min(OVERLAP_POINTS,prev_size);
  //int overlap_points = previous_path_x.size();
  //
  int consumed_points = previous_trajectory.xlocs.size() - previous_path_x.size();
  if(prev_size != 0) 
      v = previous_trajectory.velocities[consumed_points-1];
  else
      v = 0.0;
  double check_car_s, spacing;

  get_nearest_vehicles(car_s, sensor_fusion, lane, vehicle_ahead, vehicle_behind);
  if(vehicle_ahead.s < 100000) {
      double check_speed = vehicle_ahead.v;
      //check_car_s = vehicle_ahead.s + ((double) overlap_points*DT*check_speed);
      check_car_s = vehicle_ahead.s;
      spacing = check_car_s - car_s;
      
      if (spacing < 30) {
          too_close = true;
      }
  }
  
  if(!(too_close && spacing > 10.0 && intended_lane < MAX_LANES && intended_lane >= 0 && is_lane_change_safe(lane, intended_lane, car_s, sensor_fusion))) {
      return trajectory;
  }
  
  // Create a list of widely spaced (x,y) points, evenly spaced at 30 m
  // Later we will interpolate these waypoints with a spline and fill it in
  // with more points that control
  
  vector<double> ptsx;
  vector<double> ptsy;
  
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);
  
  if (prev_size < 2) {
    double prev_car_x = car_x - v*DT*cos(ref_yaw);
    double prev_car_y = car_y - v*DT*sin(ref_yaw);
  
    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);
  
    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  
  }
  else {
    ref_x = previous_path_x[overlap_points-1];
    ref_y = previous_path_y[overlap_points-1];
  
    double ref_x_prev = previous_path_x[overlap_points - 2];
    double ref_y_prev = previous_path_y[overlap_points - 2];
  
    ref_yaw =  atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
  
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }
  
  vector<double> next_wp0 = getXY(car_s+40, (2+4*intended_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s+70, (2+4*intended_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s+100, (2+4*intended_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  
  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);
  
  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
  
  for (int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
  
    ptsx[i] = (shift_x * cos(0 -  ref_yaw) - shift_y*sin(0-ref_yaw));
    ptsy[i] = (shift_x * sin(0 -  ref_yaw) + shift_y*cos(0-ref_yaw));
  }
  
  tk::spline s;
  
  s.set_points(ptsx, ptsy);
  
  for (int i = 0; i < overlap_points; i++) {
    trajectory.xlocs.push_back(previous_path_x[i]);
    trajectory.ylocs.push_back(previous_path_y[i]);
    trajectory.velocities.push_back(previous_trajectory.velocities[consumed_points+i]);
  }
  
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_distance = sqrt(target_x*target_x + target_y*target_y);
  
  double x_add_on = 0.0;
  
  for (int i = 0; i <= TRAJECTORY_POINTS-overlap_points; i++) {
    double N = (target_distance/(DT*v/MPH_TO_MS));
    double x_point  = x_add_on + target_x/N;
    double y_point  = s(x_point);
  
    x_add_on = x_point;
  
    double x_ref = x_point;
    double y_ref = y_point;
  
    x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));
  
    x_point += ref_x;
    y_point += ref_y;
  
    trajectory.xlocs.push_back(x_point);
    trajectory.ylocs.push_back(y_point);
    trajectory.velocities.push_back(v);
  }
  
  get_nearest_vehicles(car_s, sensor_fusion, intended_lane, vehicle_ahead, vehicle_behind);
  trajectory.lane_speed = vehicle_ahead.v;
  trajectory.vehicle_ahead_distance = vehicle_ahead.s - car_s;
  trajectory.intended_lane = intended_lane;
  return trajectory;
}

void Vehicle::determine_lane(float vehicle_d) {
    for(int i=0; i < MAX_LANES; i++) {
        if (vehicle_d <= (2+4*i+2) && vehicle_d > (2+4*i-2)) {
            lane = i;
        }
    }
}

float Vehicle::compute_cost(Trajectory trajectory) {
    float cost;
    if(trajectory.xlocs.size() == 0) return 1000000.0;
    if(trajectory.intended_lane == lane) {
        cost = 750.0;
    }
    else if(trajectory.intended_lane == 2) {
        cost = 500.0;
    } 
    else {
        if (trajectory.intended_lane == 0)
            cost = 250.0;
        else
            cost = 0.0;
    }
    return cost;
}

void Vehicle::get_nearest_vehicles(float ego_s, vector<vector<double>> sensor_fusion,
                                int vehicle_lane, Vehicle &vehicle_ahead, Vehicle &vehicle_behind) {
  Vehicle temp_vehicle;
  float rear_distance = 1000000;
  float front_distance = 1000000;
  vehicle_ahead.s = front_distance;
  vehicle_ahead.v = 0.0;
  vehicle_behind.s = -rear_distance;
  for (int i=0; i < sensor_fusion.size(); i++) {
    temp_vehicle.d = sensor_fusion[i][6];
    if (temp_vehicle.d < (2+4*vehicle_lane+2) && temp_vehicle.d > (2+4*vehicle_lane-2)) {
        temp_vehicle.s = sensor_fusion[i][5];
        float difference_s = temp_vehicle.s - ego_s;
        if(difference_s <= 0.0 && abs(difference_s) < rear_distance) {
            rear_distance = abs(difference_s);
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            vehicle_behind.v = sqrt(vx*vx + vy*vy);
            vehicle_behind.s = temp_vehicle.s;
        }
        if(difference_s > 0.0 && abs(difference_s) < front_distance) {
            front_distance = abs(difference_s);
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            vehicle_ahead.v = sqrt(vx*vx + vy*vy);
            vehicle_ahead.s = temp_vehicle.s;
        }
    }
  }
}
