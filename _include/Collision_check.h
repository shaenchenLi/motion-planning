#ifndef COLLISION_CHECK_H
#define COLLISION_CHECK_H

#include "core/core.hpp"
#include <cv.h>
#include <highgui.h>
using namespace cv;

#include "Environment.h"
#include "Vehicle.h"

namespace Collision
{
	Mat* maptoMat(Environment::EnvironMap *environmap);

	struct collision
	{
		collision()
		{
			kernel = nullptr;
			collision_map = nullptr;
			space = SAFESTEP;
		}
		collision(Environment::EnvironMap *environmap)
		{
			collision_map = nullptr;
			_collision_map(environmap);
			space = SAFESTEP;// depending on the length of the vehicle and the circle
		}
		~collision()
		{
			delete collision_map;
			delete kernel;
		}

		void _collision_map(Environment::EnvironMap *environmap);
		Mat* _kernel(const double &interval);
		bool iscollision(const Vehicle::Node &begin_node, const Vehicle::Node &end_node);
		bool iscollision(const double &x, const double &y, const double &theta);

		Mat* _collision_map() { return collision_map; }

	private:
		Mat* collision_map;
		Mat* kernel;
		double interval;
		double origin[2];
		double space;
	};
}

#endif