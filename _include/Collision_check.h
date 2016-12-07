#ifndef COLLISION_CHECK_H
#define COLLISION_CHECK_H

#include "core/core.hpp"
#include <cv.h>
#include <highgui.h>
using namespace cv;

#include "Environment.h"
#include "Vehicle.hpp"

namespace Collision
{
	Mat* maptoMat(Environment::EnvironMap *environmap);

	struct collision
	{
		collision()
		{
			kernel = nullptr;
			collision_map = nullptr;
		}
		collision(Environment::EnvironMap *environmap)
		{
			interval = environmap->_interval();
			kernel = _kernel(interval);
			origin[0] = environmap->_range()->begin()->x;
			origin[1] = environmap->_range()->begin()->y;
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
		Mat* _kernel(const float &interval);
		bool iscollision(const Vehicle::Node &begin_node, const Vehicle::Node &end_node);
		bool iscollision(const float &x, const float &y, const float &theta);

		Mat* _collision_map() { return collision_map; }

	private:
		Mat* collision_map;
		Mat* kernel;
		float interval;
		float origin[2];
		float space;
	};
}

#endif