//Version 5.1 Liyishan

#include <chrono>
#include <fstream>
#include <sstream>
using namespace std::chrono;

#include "Plan.h"

int main()
{
	Environment::position xymin(XMIN, YMIN), xymax(XMAX, YMAX);
	Environment::EnvironMap environmap(xymin, xymax);
	Environment::map_construct(&environmap);

	Collision::collision* collimap = new Collision::collision(&environmap);
	
	VectorXd xi(6), xg(5);
	xi << 0.5, 0., 0., 0., 4., 0.;
	xg << 3.5, 6.85, -PI, 0., 4.;
	Vector3d bound;
	bound << xi[4], xi[5], xg[4];
	Trajectory::State XI(xi);
	Trajectory::State XG(xg);

	_plan(3, XI, XG, xi(5));
}