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

if (prev_size > 0) {
  car_s = end_path_s;
}


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
string state = "KL";
auto trajectory = ego.generate_trajectory(state, sensor_fusion);

next_x_vals = trajectory.xlocs;
next_y_vals = trajectory.ylocs;
//for(int i=0; i < next_x_vals.size(); i++) {
//  std::cout<<next_x_vals[i]<< " " << next_y_vals[i] << std::endl;
//
//}

//for (int i=0; i < nstates; i++) {
//    auto trajectory = ego.generate_trajectory(state, sensor_fusion, trajectory_x, trajectory_y);
    //double cost = ego.compute_cost(sensor_fusion);
    //if (cost < min_cost) {
    //    double best_cost = cost;
    //    vector<double> best_trajectory_x = trajectory_x;
    //    vector<double> best_trajectory_y = trajectory_y;
    //    best_state = i;
    //}
//}


//std::cout << "Next Vals:" << std::endl;
//for (int i = 0; i < next_x_vals.size(); i++) {
//  std::cout << next_x_vals[i] << " " << next_y_vals[i] << std::endl;
//}

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

double goal_distance_cost(int goal_lane, int intended_lane, int final_lane, 
                          double distance_to_goal) {
  // The cost increases with both the distance of intended lane from the goal
  //   and the distance of the final lane from the goal. The cost of being out 
  //   of the goal lane also becomes larger as the vehicle approaches the goal.
  int delta_d = 2.0 * goal_lane - intended_lane - final_lane;
  double cost = 1 - exp(-(std::abs(delta_d) / distance_to_goal));

  return cost;
}

// Initializes Vehicle
Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, float s, float v, float a, string state) {
  this->lane = lane;
  this->s = s;
  this->ref_vel = v;
  this->a = a;
  this->state = state;
  max_acceleration = -1;
}

Vehicle::~Vehicle() {}

//vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> &predictions) {
//  /**
//   * Here you can implement the transition_function code from the Behavior 
//   *   Planning Pseudocode classroom concept.
//   *
//   * @param A predictions map. This is a map of vehicle id keys with predicted
//   *   vehicle trajectories as values. Trajectories are a vector of Vehicle 
//   *   objects representing the vehicle at the current timestep and one timestep
//   *   in the future.
//   * @output The best (lowest cost) trajectory corresponding to the next ego 
//   *   vehicle state.
//   *
//   * Functions that will be useful:
//   * 1. successor_states - Uses the current state to return a vector of possible
//   *    successor states for the finite state machine.
//   * 2. generate_trajectory - Returns a vector of Vehicle objects representing 
//   *    a vehicle trajectory, given a state and predictions. Note that 
//   *    trajectory vectors might have size 0 if no possible trajectory exists 
//   *    for the state. 
//   * 3. calculate_cost - Included from cost.cpp, computes the cost for a trajectory.
//   *
//   * TODO: Your solution here.
//   */
//  vector<string> states = successor_states();
//  float cost;
//  vector<float> costs;
//  vector<vector<Vehicle>> final_trajectories;
//
//  for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
//    vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
//    if (trajectory.size() != 0) {
//      cost = calculate_cost(*this, predictions, trajectory);
//      costs.push_back(cost);
//      final_trajectories.push_back(trajectory);
//    }
//  }
//
//  vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
//  int best_idx = distance(begin(costs), best_cost);
//
//  /**
//   * TODO: Change return value here:
//   */
//  return final_trajectories[best_idx];
//}

//vector<string> Vehicle::successor_states() {
//  // Provides the possible next states given the current state for the FSM 
//  //   discussed in the course, with the exception that lane changes happen 
//  //   instantaneously, so LCL and LCR can only transition back to KL.
//  vector<string> states;
//  states.push_back("KL");
//  string state = this->state;
//  if(state.compare("KL") == 0) {
//    states.push_back("PLCL");
//    states.push_back("PLCR");
//  } else if (state.compare("PLCL") == 0) {
//    if (lane != lanes_available - 1) {
//      states.push_back("PLCL");
//      states.push_back("LCL");
//    }
//  } else if (state.compare("PLCR") == 0) {
//    if (lane != 0) {
//      states.push_back("PLCR");
//      states.push_back("LCR");
//    }
//  }
//    
//  // If state is "LCL" or "LCR", then just return "KL"
//  return states;
//}

Trajectory Vehicle::generate_trajectory(string state, vector<vector<double>> sensor_fusion) {
  // Given a possible next state, generate the appropriate trajectory to realize
  //   the next state.
  Trajectory trajectory;
  if (state.compare("CS") == 0) {
    trajectory = constant_speed_trajectory();
  } else if (state.compare("KL") == 0) {
    trajectory = keep_lane_trajectory(state, sensor_fusion);
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    trajectory = lane_change_trajectory(state, sensor_fusion);
  } 

  return trajectory;
}

bool Vehicle::is_lane_change_safe(int current_lane, int intended_lane, float ego_s, vector<vector<double>> sensor_fusion) {
    double front_margin = 10.0;
    double rear_margin = 20.0;
    Vehicle vehicle_ahead, vehicle_behind;
    get_nearest_vehicles(ego_s, sensor_fusion, intended_lane, vehicle_ahead, vehicle_behind);
    if(vehicle_ahead.s - ego_s > front_margin && ego_s - vehicle_behind.s > rear_margin)
        return true;
    else
        return false;
}

//vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> &predictions, 
//                                      int lane) {
//  // Gets next timestep kinematics (position, velocity, acceleration) 
//  //   for a given lane. Tries to choose the maximum velocity and acceleration, 
//  //   given other vehicle positions and accel/velocity constraints.
//  float max_velocity_accel_limit = this->max_acceleration + this->v;
//  float new_position;
//  float new_velocity;
//  float new_accel;
//  Vehicle vehicle_ahead;
//  Vehicle vehicle_behind;
//
//  //NOTE: Code below should be modified to use delta_t=0.02 instead of delta_t=1
//  if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
//    if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
//      // must travel at the speed of traffic, regardless of preferred buffer
//      new_velocity = vehicle_ahead.v;
//    } else {
//      float max_velocity_in_front = (vehicle_ahead.s - this->s 
//                                  - this->preferred_buffer) + vehicle_ahead.v 
//                                  - 0.5 * (this->a);
//      new_velocity = std::min(std::min(max_velocity_in_front, 
//                                       max_velocity_accel_limit), 
//                                       this->target_speed);
//    }
//  } else {
//    new_velocity = std::min(max_velocity_accel_limit, this->target_speed);
//  }
//    
//  new_accel = new_velocity - this->v; // Equation: (v_1 - v_0)/t = acceleration
//  new_position = this->s + new_velocity + new_accel / 2.0;
//    
//  return{new_position, new_velocity, new_accel};
//}

Trajectory Vehicle::constant_speed_trajectory() {
  // Generate a constant speed trajectory.
  Trajectory trajectory;
  return trajectory;
}

Trajectory Vehicle::keep_lane_trajectory(string state, vector<vector<double>> sensor_fusion) {
  // Generate a keep lane trajectory.
  Trajectory trajectory;
  int prev_size = previous_path_x.size();
  int intended_lane;
  bool too_close = false;
  double car_x = x;
  double car_y = y;
  double car_s = s;
  double car_yaw = yaw;

  intended_lane = lane;
  for (int i = 0; i < sensor_fusion.size(); i++) {
      float d = sensor_fusion[i][6];
      if (d < (2+4*lane+2) && d > (2+4*lane-2)) {
          double vx = sensor_fusion[i][3];
          double vy = sensor_fusion[i][4];
          double check_speed = sqrt(vx*vx + vy*vy);
          double check_car_s = sensor_fusion[i][5];

  
          check_car_s += ((double) prev_size*0.02*check_speed);
          if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
              //ref_vel = 29.5;
              too_close = true;
  
              if(lane + 1 < MAX_LANES && is_lane_change_safe(lane, lane+1, car_s, sensor_fusion)) {
                  intended_lane = lane+1;
              }
              else if (lane - 1 >= 0 && is_lane_change_safe(lane, lane-1, car_s, sensor_fusion)) {
                  intended_lane = lane-1;
              }
          }
      }
  }
  
  if (too_close) {
    ref_vel -= 0.224; // Decrement by about 5 m/s
    //std::cout << "decrementing ref_vel" << ref_vel;
  }
  else if (ref_vel < 49.5) {
    ref_vel += 0.224;
    //std::cout << "incrementing ref_vel" << ref_vel;
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
    double prev_car_x = car_x - ref_vel*0.02*cos(ref_yaw);
    double prev_car_y = car_y - ref_vel*0.02*sin(ref_yaw);
  
    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);
  
    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  
  }
  else {
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];
  
    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
  
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
  
  //std::cout << "ptsx, ptsy:" << std::endl;
  for (int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
  
    ptsx[i] = (shift_x * cos(0 -  ref_yaw) - shift_y*sin(0-ref_yaw));
    ptsy[i] = (shift_x * sin(0 -  ref_yaw) + shift_y*cos(0-ref_yaw));
    //std::cout << i << " : " << ptsx[i] << " " << ptsy[i] << " " << deg2rad(ref_yaw) << std::endl;
  }
  
  tk::spline s;
  
  s.set_points(ptsx, ptsy);
  
  for (int i = 0; i < previous_path_x.size(); i++) {
    trajectory.xlocs.push_back(previous_path_x[i]);
    trajectory.ylocs.push_back(previous_path_y[i]);
  }
  
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_distance = sqrt(target_x*target_x + target_y*target_y);
  
  double x_add_on = 0.0;
  
  for (int i = 0; i <= 50-previous_path_x.size(); i++) {
    double N = (target_distance/(0.02 * ref_vel/2.24));
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
  }
  
  return trajectory;
}

//vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, vector<vector<double>> sensor_fusion) {
  // Generate a trajectory preparing for a lane change.
  //float new_s;
  //float new_v;
  //float new_a;
  //Vehicle vehicle_behind;
  //int new_lane = this->lane + lane_direction[state];
  //vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, 
  //                                      this->state)};
  //vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

  //if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
  //  // Keep speed of current lane so as not to collide with car behind.
  //  new_s = curr_lane_new_kinematics[0];
  //  new_v = curr_lane_new_kinematics[1];
  //  new_a = curr_lane_new_kinematics[2];    
  //} else {
  //  vector<float> best_kinematics;
  //  vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
  //  // Choose kinematics with lowest velocity.
  //  if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
  //    best_kinematics = next_lane_new_kinematics;
  //  } else {
  //    best_kinematics = curr_lane_new_kinematics;
  //  }
  //  new_s = best_kinematics[0];
  //  new_v = best_kinematics[1];
  //  new_a = best_kinematics[2];
  //}

  //trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));

//  Trajectory trajectory;
//  
//  return trajectory;
//}

Trajectory Vehicle::lane_change_trajectory(string state, vector<vector<double>> sensor_fusion) {
  //// Generate a lane change trajectory.
  //int new_lane = this->lane + lane_direction[state];
  //vector<Vehicle> trajectory;
  //Vehicle next_lane_vehicle;
  //// Check if a lane change is possible (check if another vehicle occupies 
  ////   that spot).
  //for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); 
  //     it != predictions.end(); ++it) {
  //  next_lane_vehicle = it->second[0];
  //  if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane) {
  //    // If lane change is not possible, return empty trajectory.
  //    return trajectory;
  //  }
  //}
  //trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, 
  //                             this->state));
  //vector<float> kinematics = get_kinematics(predictions, new_lane);
  //trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], 
  //                             kinematics[2], state));

  Trajectory trajectory;
  return trajectory;
}

void Vehicle::determine_lane(float vehicle_d) {
    for(int i=0; i < MAX_LANES; i++) {
        if (vehicle_d <= (2+4*i+2) && vehicle_d > (2+4*i-2)) {
            lane = i;
        }
    }
}

//void Vehicle::increment(int dt = 1) {
//  this->s = position_at(dt);
//}
//
//float Vehicle::position_at(int t) {
//  return this->s + this->v*t + this->a*t*t/2.0;
//}
//
//bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> &predictions, 
//                                 int lane, Vehicle &rVehicle) {
//  // Returns a true if a vehicle is found behind the current vehicle, false 
//  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
//  int max_s = -1;
//  bool found_vehicle = false;
//  Vehicle temp_vehicle;
//  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); 
//       it != predictions.end(); ++it) {
//    temp_vehicle = it->second[0];
//    if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s 
//        && temp_vehicle.s > max_s) {
//      max_s = temp_vehicle.s;
//      rVehicle = temp_vehicle;
//      found_vehicle = true;
//    }
//  }
//  
//  return found_vehicle;
//}

void Vehicle::get_nearest_vehicles(float ego_s, vector<vector<double>> sensor_fusion,
                                int vehicle_lane, Vehicle &vehicle_ahead, Vehicle &vehicle_behind) {
  Vehicle temp_vehicle;
  float rear_distance = 1000000;
  float front_distance = 1000000;
  vehicle_ahead.s = front_distance;
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
            vehicle_behind.ref_vel = sqrt(vx*vx + vy*vy);
            vehicle_behind.s = temp_vehicle.s;
        }
        if(difference_s > 0.0 && abs(difference_s) < front_distance) {
            front_distance = abs(difference_s);
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            vehicle_ahead.ref_vel = sqrt(vx*vx + vy*vy);
            vehicle_ahead.s = temp_vehicle.s;
        }
    }
  }
}

//  return found_vehicle;
//}
//
//vector<Vehicle> Vehicle::generate_predictions(int horizon) {
//  // Generates predictions for non-ego vehicles to be used in trajectory 
//  //   generation for the ego vehicle.
//
//  //   NOTE: predictions based on delet_t=1, needs to be modified
//  vector<Vehicle> predictions;
//  for(int i = 0; i < horizon; ++i) {
//    float next_s = position_at(i);
//    float next_v = 0;
//    if (i < horizon-1) {
//      next_v = position_at(i+1) - s;
//    }
//    predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
//  }
//  
//  return predictions;
//}
//
//void Vehicle::realize_next_state(vector<Vehicle> &trajectory) {
//  // Sets state and kinematics for ego vehicle using the last state of the trajectory.
//  Vehicle next_state = trajectory[1];
//  this->state = next_state.state;
//  this->lane = next_state.lane;
//  this->s = next_state.s;
//  this->v = next_state.v;
//  this->a = next_state.a;
//}
//
//void Vehicle::configure(vector<int> &road_data) {
//  // Called by simulator before simulation begins. Sets various parameters which
//  //   will impact the ego vehicle.
//  target_speed = road_data[0];
//  lanes_available = road_data[1];
//  goal_s = road_data[2];
//  goal_lane = road_data[3];
//  max_acceleration = road_data[4];
//}
