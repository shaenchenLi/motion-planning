//Version 4.1 Liyishan

#include <chrono>
#include <fstream>
#include <sstream>
using namespace std::chrono;

#include "Collision_check.h"
#include "Environment.h"
#include "RRT.h"
#include "Trajectory.h"

int main()
{
	// 生成数组用于确保曲线的曲率
	vector<double> *L_theta = new vector<double>;
	Vehicle::ensure_k(L_theta);

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

	int count = 0;
	auto start = steady_clock::now();
	RRT::RTG_RRT rrt(*XI._node(), *XG._node(), 1);
	int result = rrt.search(3, L_theta, collimap, &count);
	auto end = steady_clock::now();

	if (result)
	{
		auto duration = end - start;
		rrt.getpath();
		Trajectory::traj trajectory(xi, xg);
		if (result == 2)
			trajectory._bspline(*rrt._route_tree());
		else
		{
			trajectory._ctrl_points(*rrt._route_tree(), L_theta, collimap);
			trajectory._bspline();
		}

		trajectory._state(&(xi[5]));
		vector<Trajectory::State> states = *trajectory._state_future();
	}
}