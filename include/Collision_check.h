#ifndef COLLISION_CHECK_H
#define COLLISION_CHECK_H

#include <chrono>
using namespace std::chrono;

#include "Environment.h"
//#include "Vehicle.h"

extern Environment::time_EnvironMap dynamic_environmap;

namespace Collision
{
	//true:unsafe  false:safe
	/******************************************¾²Ì¬Åö×²¼ì²â*******************************************/
	bool iscollision(const Vehicle::Node &begin_node, const Vehicle::Node &end_node);
	bool iscollision(const double &x, const double &y, const double &theta);
	bool iscollision(const double &x, const double &y);

	/******************************************¶¯Ì¬Åö×²¼ì²â*******************************************/
	bool is_dynamic_collision(const Vehicle::Node &node, const double &t);
	bool is_dynamic_collision(const double &x, const double &y, const double &theta, const double &t);
	bool is_dynamic_collision(const double &x, const double &y, const double &t);

	bool _safe_time_point(const Vehicle::Node &node, const double &t_min, double *t_safe, const double t_max = dynamic_environmap._t_range());
}

#endif