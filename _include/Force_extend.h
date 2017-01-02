#ifndef FORCE_EXTEND_H
#define FORCE_EXTEND_H

#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>
using std::cout; using std::endl;
using std::vector;

#include "tinysplinecpp.h"

#include "Collision_check.h"
#include "Parameters.h"
#include "Vehicle.hpp"
#include "Trajectory.h"

namespace Trajectory
{
	//flag=1:lanechangeleft;flag=2:lanechangeright;flag=3:lanechangeleftc;flag=4:lanechangerightc
	//flag=5:leftturn;flag=6:rightturn;flag=7:leftturnc;flag=8:Uturn
	bool straight(const vector<double> &bound_condition, vector<double> *control);
	bool lanechange(const int &sign, const vector<double> &L_theta, const Vehicle::Node &start_node, const Vehicle::Node &end_node, vector<double> *control);
	bool lanechange_c(const int &sign, const vector<double> &L_theta, const Vehicle::Node &start_node, const Vehicle::Node &end_node, const vector<double> &constraints, vector<double> *control);
	bool turn(const int &sign, const vector<double> &L_theta, const Vehicle::Node &start_node, const Vehicle::Node &end_node, vector<double> *control);
	bool turn_c(const int &sign, const vector<double> &L_theta, const Vehicle::Node &start_node, const Vehicle::Node &end_node, const vector<double> &constraints, vector<double> *control);
	bool U_turn(const vector<double> &L_theta, const Vehicle::Node &start_node, const Vehicle::Node &end_node, vector<double> *control);
	bool U_turn_c(const vector<double> &L_theta, const Vehicle::Node &start_node, const Vehicle::Node &end_node, const vector<double> &constraints, vector<double> *control);
	int is_force_extend_safe(const Vehicle::Node &startnode, const double &step, const vector<double> &control, Collision::collision *collimap, double *le, vector<Vehicle::Node> *path);
}

#endif