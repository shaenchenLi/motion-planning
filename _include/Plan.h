#ifndef PLAN_H
#define PLAN_H

#include <chrono>
using namespace std::chrono;

#include "Collision_check.h"
#include "Force_extend.h"
#include "Parameters.h"
#include "RRT.h"
#include "Trajectory.h"
#include "Vehicle.h"

void _plan(const int &type, const Trajectory::State &state_i, const Trajectory::State &state_g, const double &accel, const double l = 0., const double w = 0., vector<Trajectory::State> *adjust_states_front = nullptr, vector<Trajectory::State> *adjust_states_end = nullptr);

#endif