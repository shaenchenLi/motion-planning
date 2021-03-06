#ifndef FORCE_EXTEND_H
#define FORCE_EXTEND_H

#include <algorithm>

#include "tinysplinecpp.h"
#include <Dense>
using namespace Eigen;
using Vector5d = Matrix<double, 5, 1>;
using Vector6d = Matrix<double, 6, 1>;
using Vector7d = Matrix<double, 7, 1>;

#include "Collision_check.h"
#include "Vehicle.h"
#include "Trajectory.h"

namespace Trajectory
{
	//flag=1:lanechangeleft;flag=2:lanechangeright;flag=3:lanechangeleftc;flag=4:lanechangerightc
	//flag=5:leftturn;flag=6:rightturn;flag=7:leftturnc;flag=8:Uturn
	//angle表示车辆初始点处车道中心线（全局路径）与大地坐标系的夹角
	int orient_trans(const Vehicle::Node &start_node, const Vehicle::Node &end_node, const double &angle, Vehicle::Node *local_start_node, Vehicle::Node *local_end_node); //返回车辆是向左还是向右,右1左-1
	bool straight(const Vehicle::Node &start_node, const Vehicle::Node &end_node, vector<double> *control);
	bool lanechange(const Vehicle::Node &start_node, const Vehicle::Node &end_node, const double &angle, vector<double> *control);
	bool lanechange_c(const Vehicle::Node &start_node, const Vehicle::Node &end_node, const double &angle, const vector<double> &constraints, vector<double> *control);
	bool lanechange_r_c(const Vehicle::Node &start_node, const Vehicle::Node &end_node, const double &angle, const vector<double> &constraints, vector<double> *control);
	bool turn(const Vehicle::Node &start_node, const Vehicle::Node &end_node, const double &angle, vector<double> *control);
	bool turn_c(const Vehicle::Node &start_node, const Vehicle::Node &end_node, const double &angle, const vector<double> &constraints, vector<double> *control);
	bool U_turn(const Vehicle::Node &start_node, const Vehicle::Node &end_node, const double &angle, vector<double> *control);
	bool U_turn_c(const Vehicle::Node &start_node, const Vehicle::Node &end_node, const double &angle, const vector<double> &constraints, vector<double> *control);
	int is_force_extend_safe(const Vehicle::Node &startnode, const Vehicle::Node &startnode_r, const double &step, const vector<double> &control, double *le, vector<Vehicle::Node> *path);
}

#endif