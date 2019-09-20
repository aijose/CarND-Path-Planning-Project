#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>
#include "helpers.h"
#include <math.h>
#include <algorithm>
#include "spline.h"

#define MAX_LANES 3

using std::map;
using std::string;
using std::vector;

struct Trajectory {
    vector<double> xlocs;
    vector<double> ylocs;
    vector<double> velocities;
    float vehicle_ahead_distance;
    float lane_speed;
    int intended_lane;
};

class Vehicle {
 public:
  // Constructors
  Vehicle();
  Vehicle(int d, float s, float v, float a, string state="CS");

  // Destructor
  virtual ~Vehicle();

  vector<string> successor_states();

  Trajectory generate_trajectory(string state, vector<vector<double>> sensor_fusion);

  Trajectory constant_speed_trajectory();

  Trajectory keep_lane_trajectory(vector<vector<double>> sensor_fusion);

  Trajectory lane_change_trajectory(int intended_lane, vector<vector<double>> sensor_fusion);

  float compute_cost(Trajectory trajectory);

  void determine_lane(float car_d);

  void get_nearest_vehicles(float ego_s, vector<vector<double>> sensor_fusion,
                                int vehicle_lane, Vehicle &vehicle_ahead, Vehicle &vehicle_behind);

  void get_lane_speed(float ego_s, int vehicle_lane, vector<vector<double>> sensor_fusion);

  bool is_lane_change_safe(int current_lane, int intended_lane, float ego_s, vector<vector<double>> sensor_fusion);

  map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, 
                                     {"LCR", -1}, {"PLCR", -1}};

  //int L = 1;

  //int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane;
  double x, y, d, s, yaw;

  float ref_vel, target_speed, a, max_acceleration;
  vector<double> previous_path_x, previous_path_y;
  vector<double> map_waypoints_s, map_waypoints_x, map_waypoints_y;
  Trajectory previous_trajectory;

  string state;
};

#endif  // VEHICLE_H
