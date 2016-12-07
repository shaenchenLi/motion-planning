#ifndef FORCE_EXTEND_H
#define FORCE_EXTEND_H

#include <algorithm>
#include <vector>
using std::vector;

#include "tinysplinecpp.h"

#include "Collision_check.h"
#include "Parameters.h"
#include "Vehicle.hpp"

namespace Trajectory
{
	//flag=1:lanechangeleft;flag=2:lanechangeright;flag=3:lanechangeleftc;flag=4:lanechangerightc
	//flag=5:leftturn;flag=6:rightturn;flag=7:leftturnc;flag=8:Uturn
	bool lanechange(const int &sign, const vector<float> &L_theta, const vector<float> &bound_condition, vector<float> *control);
	bool lanechange_c(const int &sign, const vector<float> &L_theta, const vector<float> &bound_condition, const vector<float> &constraints, vector<float> *control);
	bool turn(const int &sign, const vector<float> &L_theta, const vector<float> &bound_condition, vector<float> *control);
	bool turn_c(const vector<float> &L_theta, const vector<float> &bound_condition, const vector<float> &constraints, vector<float> *control);
	bool U_turn(const vector<float> &L_theta, const vector<float> &bound_condition, vector<float> *control);
	bool U_turn_c(const vector<float> &L_theta, const vector<float> &bound_condition, const vector<float> &constraints, vector<float> *control);
	int is_force_extend_safe(const Vehicle::Node &startnode, const float &step, const vector<float> &control, Collision::collision *collimap, float *le, vector<Vehicle::Node> *path);
}

#endif