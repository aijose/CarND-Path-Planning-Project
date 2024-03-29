# **Highway Driving**

---

## Objectives

The goals / steps of this project are the following:
* Design a path planner that is able to create smooth, safe paths for the car to follow along a three lane highway with traffic
* The following events should be avoided:
  * Collision with another vehicle
  * Exceeding the maximum speed of 50 mph
  * Acceleration greater than 10 m/sq-sec
  * Jerk of 10 m/cu-s
  * Straying outside the marked lanes except while changing lanes
* Test that the model successfully drives around track (4.32 miles) without any incidents
* Summarize the solution approach with a written report

## Rubric Points
### This section will describe how the each of the [rubric points](https://review.udacity.com/#!/rubrics/1971/view) were addressed in this project.  

---

### Overview of Methodology

The solution approach uses a Finite State Machine (FSM) to model the behavior of
the car. Only three states are used:
1. Keep lane state (KL)
2. Lane Change Left (LCL)
2. Lane Change Right (LCR)

The transitions between states in the FSM are shown in the image below:

<p align="center">
<img src="fsm.png" width="1000%" alt>
</p>
<p align="center">
<em> Finite State Machine </em>
</p>

The car starts off in the KL state. The KL state remain in the same state or
transition to the LCL and LCR states. The LCL and LCR states
can either remain in the same state or transition to the KL state.

Every time the sensor fusion measurements are made available, the vehicle
generates trajectories for all possible successor states. For each successor
state, the cost of the trajectory is evaluated based on a cost function. The
state corresponding to the trajectory with the least cost is chosen as the next
state. The trajectory corresponding to the next state is provided to the simulator.
The following code snippet shows the basic logic for choosing the best trajectory.

```cpp
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
```

The following sub-sections describe the different states.

#### Keep Lane State

In this state, the ego vehicle sticks to its current lane. The key task in this state is
controlling the velocity. Once the velocity is known, the trajectory can be
easily determined by incrementally updating the vehicle position. For this purpose,
the vehicle can be assumed to be in three "sub-states":

1. "Very close" to the vehicle ahead (corresponding to a distance less than 10 units). In this state, the vehicle velocity is decreased more drastically
and linearly proportionate to the distance from the vehicle ahead. This sub-state
is particularly important when a car from an adjacent lane abruptly changes into
ego vehicle's lane at a very close distance. Rapid deceleration is critical to
avoid collision in such situations.

2. "Too close" to the vehicle ahead (corresponding to a distance between 10 and 30 units). In this state, the vehicle velocity is decreased less drastically and linearly proportionate to the distance from the vehicle ahead. Additionally, the
ego vehicle's velocity is reduced only if its velocity is greater than the
vehicle ahead.

3. "Far away" from the vehicle ahead (corresponding to a distance greater than
30 units). In this state, the vehicle velocity is increased at a constant
acceleration until the vehicle reaches a velocity of 49 mph beyond which it is
keep constant.

Trajectory generation for the KL state  is described in more detail in a later
section on trajectory generation.

#### Lane Change Left and Lane Change Right States

In the LCL and LCR states, the vehicle speed is kept at the same velocity as
when it entered these states. Since logic is the same for LCL and LCR, both
states implemented in the same function with the target lane as one of the
inputs. An important decision for LCL and LCR states is when to change lanes.
The following considerations had to be satisfied for lane change.

1. Separation from the vehicle ahead (in the same lane) must be less than 30
units but greater than 10 units of length. This is a simplifying assumption that
also makes it easier when designing the cost function.

2. Separation from vehicles ahead should satisfy the front margin distances
(35-40 units). The front margin is kept high so that the vehicle does not enter
the "too close state" (described in an earlier section) thereby necessitating
abrupt velocity reductions in addition to lane change related jerk.

3. Separation from vehicles behind the vehicle in the target lane must satisfy the
rear margin distance (35 units) plus a speed buffer distance that increases when the vehicle velocity is less than the velocity of the vehicle behind it. The speed buffer
is zero when the vehicle behind is traveling at a slower speed.

The parameters used to define the front margin, rear margin and speed buffer distance are chosen conservatively to avoid failures in even in uncommon scenarios. The code
snippet that checks for safe lane change are provided below:

```cpp
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
```

Once it is decided that lane change is safe, the next task for the LCL and LCR
states is to create the trajectory for  the lane change. This is described in more detail in a later section.

#### Cost Function

The generated trajectories are evaluated using a cost function. For the purposes
of this project, a very simple cost function was used. A simple cost function
was adequate because feasibility and safety of the trajectory is already ensured
before the cost function is evaluated.

Firstly, if a lane change is at all feasible and safe, then it is assumed that
the lane change trajectory has a lower cost than the keep lane trajectory.
The keep lane trajectory is given the highest cost of 750. If a lane change is
possible to the left or right lanes, then the left lane is preferred because
the track is anti-clockwise and therefore the inner lane is likely to involve
less overall distance. A more complex cost function may prove to be more
efficient, but may also result in situations where the vehicle trajectory
oscillates between two lanes leading to unexpected behavior. The cost function
code is provided below:

```cpp
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
```

### Compilation

#### 1. The code compiles correctly

The project submission includes the following files:
* src/main.cpp containing the main code
* src/vehicle.h contains the vehicle class description
* src/spline.h contains the code for the spline function used to generate trajectories
* project_report.md containing the writeup for this project

### Validation Trajectories

#### 1. The car is able to drive at least 4.32 miles without incident

The final model was able to complete 25 miles without incident.


#### 2. The car drives according to the speed limit

As noted earlier in the description for the keep lane state, there is a check
that ensures that the car accelerates only if the velocity of the car is less
than 49 mph. This ensures that the car speed does not exceed the speed limit of
50 mph.

#### 3. Max acceleration and jerk are not exceeded

The acceleration is always applied in small increments that ensure that the max
acceleration and jerk are not exceeded. There could be additional source of
acceleration due to the curvature of the track and the vehicle trajectories
during lane changes, but these are not significant  enough to exceed to
acceleration and jerk thresholds.

#### 4. Car does not have collisions

As described earlier in the earlier sections describing the keep lane and lane change states, the vehicle velocity is reduced when it comes too close to other vehicles and appropriate safety margins are maintained during lane changes.
The final code was able to travel 25 miles of the track without any collisions.

#### 5. The car stays in its lane, except for the time between changing lanes

When the car is in the keep lane state, its Frenet coordinate, d, is kept
constant and this automatically ensures that the car remains in its lane. The
car completes the lane change state fairly quickly. One issue that could arise
is that the car could keep oscillating between the lane change and keep lane states. This could lead to scenarios where the car stays outside
the lane for too long. The parameters for the lane change process were chosen
such that such a behavior does not persist. For example, by ensuring that the
front margin during lane change is greater than the distance that triggers lane
change we can prevent the car from entering the lane change state as soon as it
enters the next lane.

#### 6. The car is able to change lanes

The conditions for lane change have been described earlier and while these may
be conservative, they typically trigger lane changes several times during a single lap
around the track.

### Reflection on how to generate paths

As noted earlier, paths can be generated in the keep lane (KL) state and the
lane change (LCL and LCR) states. It involves two components, the shape of the
curve and the spacing of points in the curve. To ensure continuity with  the
previous trajectory, a fixed number of points from the previous trajectory
(known as overlap_points) is added to the current trajectory. In the current
project, the number of overlap points from the previous trajectory was set to a
minimal value of 2 to allow for more dynamic responses to changing environments.

A trajectory shape is generated by using a spline function. To maintain  continuity
with the previous trajectory, the last two way points from the previous
trajectory  are added to the list of spline points. Three further way points are
added to the spline points list. These points are some distance away (s=40, 70
and 100  units of length) in the intended target lane of the trajectory. The
aforementioned three points ensure that the shape of the trajectory ends as a
straight line in the intended target lane of the trajectory. The spline function
is generated using the above five points and is used to define the shape of the
trajectory. Once the shape is defined by the spline function, the trajectory can
be created by adding points along the shape. The spacing of the points is
determined by the local velocity. For the lane change trajectory, the velocity
is constant. For the keep lane trajectory, the spacing is affected by accelerations
and decelerations in velocity depending on the distance of the vehicle from the
vehicle immediately ahead of it.

The trajectory generation code for the keep lane trajectory is shown below. The
trajectory generation code for the lane change trajectory is simpler since the
velocity is kept constant during lane change:

```cpp
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
```

### Video of Final Result

The video showing successful completion of 10 miles (more than 2 laps around the track) without any incident is provided below:

[![IMAGE ALT TEXT HERE](youtube.png)](https://youtu.be/RhxwWp9stts)
