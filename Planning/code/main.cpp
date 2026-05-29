/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 *            Aaron Brown
 **********************************************/

/**
 * @file main.cpp
 **/

#include <string>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include <fstream>
#include <typeinfo>
#include <limits>
#include <time.h>
#include <math.h>

#include "json.hpp"
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include "Eigen/QR"
#include "behavior_planner_FSM.h"
#include "motion_planner.h"
#include "planning_params.h"
#include "utils.h"
#include <uWS/uWS.h>

using namespace std;
using json = nlohmann::json;

#define _USE_MATH_DEFINES

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("{");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) { return ""; }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

double angle_between_points(double x1, double y1, double x2, double y2) {
  return atan2(y2 - y1, x2 - x1);
}

// ---------------------------------------------------------------------------
// Global planner instances
// ---------------------------------------------------------------------------
BehaviorPlannerFSM behavior_planner(
    P_LOOKAHEAD_TIME, P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX, P_SPEED_LIMIT,
    P_STOP_THRESHOLD_SPEED, P_REQ_STOPPED_TIME, P_REACTION_TIME,
    P_MAX_ACCEL, P_STOP_LINE_BUFFER);

MotionPlanner motion_planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);

bool have_obst = false;
vector<State> obstacles;

// ---------------------------------------------------------------------------
// path_planner
// ---------------------------------------------------------------------------
void path_planner(vector<double>& x_points, vector<double>& y_points,
                  vector<double>& v_points, double yaw, double velocity,
                  State goal, bool is_junction, string tl_state,
                  vector<vector<double>>& spirals_x,
                  vector<vector<double>>& spirals_y,
                  vector<vector<double>>& spirals_v,
                  vector<int>& best_spirals) {

  State ego_state;
  ego_state.location.x = x_points[x_points.size()-1];
  ego_state.location.y = y_points[y_points.size()-1];
  ego_state.velocity.x = velocity;

  if (x_points.size() > 1) {
    ego_state.rotation.yaw = angle_between_points(
        x_points[x_points.size()-2], y_points[y_points.size()-2],
        x_points[x_points.size()-1], y_points[y_points.size()-1]);
    ego_state.velocity.x = v_points[v_points.size()-1];
    if (velocity < 0.01)
      ego_state.rotation.yaw = yaw;
  }

  Maneuver behavior = behavior_planner.get_active_maneuver();
  goal = behavior_planner.state_transition(ego_state, goal, is_junction, tl_state);

  if (behavior == STOPPED) {
    int max_points = 20;
    double point_x = x_points[x_points.size()-1];
    double point_y = y_points[y_points.size()-1];
    while ((int)x_points.size() < max_points) {
      x_points.push_back(point_x);
      y_points.push_back(point_y);
      v_points.push_back(0);
    }
    return;
  }

  auto goal_set      = motion_planner.generate_offset_goals(goal);
  auto spirals       = motion_planner.generate_spirals(ego_state, goal_set);
  auto desired_speed = utils::magnitude(goal.velocity);
  State lead_car_state;

  if (spirals.size() == 0) {
    cout << "Error: No spirals generated" << endl;
    return;
  }

  for (int i = 0; i < (int)spirals.size(); i++) {
    auto trajectory = motion_planner._velocity_profile_generator
        .generate_trajectory(spirals[i], desired_speed, ego_state,
                             lead_car_state, behavior);
    vector<double> spiral_x, spiral_y, spiral_v;
    for (int j = 0; j < (int)trajectory.size(); j++) {
      spiral_x.push_back(trajectory[j].path_point.x);
      spiral_y.push_back(trajectory[j].path_point.y);
      spiral_v.push_back(trajectory[j].v);
    }
    spirals_x.push_back(spiral_x);
    spirals_y.push_back(spiral_y);
    spirals_v.push_back(spiral_v);
  }

  best_spirals = motion_planner.get_best_spiral_idx(spirals, obstacles, goal);
  int best_spiral_idx = -1;
  if (best_spirals.size() > 0)
    best_spiral_idx = best_spirals[best_spirals.size()-1];

  int index = 0;
  int max_points = 20;
  int add_points = (int)spirals_x[best_spiral_idx].size();
  while ((int)x_points.size() < max_points && index < add_points) {
    x_points.push_back(spirals_x[best_spiral_idx][index]);
    y_points.push_back(spirals_y[best_spiral_idx][index]);
    v_points.push_back(spirals_v[best_spiral_idx][index]);
    index++;
  }
}

// ---------------------------------------------------------------------------
// set_obst
// ---------------------------------------------------------------------------
void set_obst(vector<double> x_points, vector<double> y_points,
              vector<State>& obstacles, bool& obst_flag) {
  for (int i = 0; i < (int)x_points.size(); i++) {
    State obstacle;
    obstacle.location.x = x_points[i];
    obstacle.location.y = y_points[i];
    obstacles.push_back(obstacle);
  }
  obst_flag = true;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main() {
  cout << "starting server" << endl;
  uWS::Hub h;

  time_t prev_timer, timer;
  time(&prev_timer);
  int i = 0;

  h.onMessage([&timer, &prev_timer, &i](
      uWS::WebSocket<uWS::SERVER> ws, char* data,
      size_t length, uWS::OpCode opCode) {

    auto s = hasData(data);
    if (s == "") return;

    auto data_j = json::parse(s);

    // ---- Unpack simulator message -----------------------------------------
    vector<double> x_points = data_j["traj_x"];
    vector<double> y_points = data_j["traj_y"];
    vector<double> v_points = data_j["traj_v"];
    double yaw        = data_j["yaw"];
    double velocity   = data_j["velocity"];
    double waypoint_x = data_j["waypoint_x"];
    double waypoint_y = data_j["waypoint_y"];
    double waypoint_t = data_j["waypoint_t"];
    bool   is_junction = data_j["waypoint_j"];
    string tl_state    = data_j["tl_state"];

    if (!have_obst) {
      vector<double> x_obst = data_j["obst_x"];
      vector<double> y_obst = data_j["obst_y"];
      set_obst(x_obst, y_obst, obstacles, have_obst);
    }

    State goal;
    goal.location.x   = waypoint_x;
    goal.location.y   = waypoint_y;
    goal.rotation.yaw = waypoint_t;

    vector<vector<double>> spirals_x, spirals_y, spirals_v;
    vector<int> best_spirals;

    path_planner(x_points, y_points, v_points, yaw, velocity, goal,
                 is_junction, tl_state,
                 spirals_x, spirals_y, spirals_v, best_spirals);

    // ---- Send response ----------------------------------------------------
    json msgJson;

    // Throttle and steer are hardcoded for the planning project.
    // Real PID control is implemented in nd013-c6-control-starter.
    msgJson["throttle"] = 0.25;
    msgJson["steer"]    = 0.0;

    msgJson["trajectory_x"] = x_points;
    msgJson["trajectory_y"] = y_points;
    msgJson["trajectory_v"] = v_points;
    msgJson["spirals_x"]    = spirals_x;
    msgJson["spirals_y"]    = spirals_y;
    msgJson["spirals_v"]    = spirals_v;
    msgJson["spiral_idx"]   = best_spirals;
    msgJson["active_maneuver"] = behavior_planner.get_active_maneuver();

    // min point threshold before requesting a new plan
    // 19 = high update rate, 4 = low update rate
    msgJson["update_point_thresh"] = 16;

    auto msg = msgJson.dump();
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

    i++;
  });

  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    cout << "Connected!!!" << endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char* message, size_t length) {
    ws.close();
    cout << "Disconnected" << endl;
  });

  int port = 4567;
  if (h.listen("0.0.0.0", port)) {
    cout << "Listening to port " << port << endl;
    h.run();
  } else {
    cerr << "Failed to listen to port" << endl;
    return -1;
  }
}
