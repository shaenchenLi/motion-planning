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
	vector<float> *L_theta = new vector<float>;
	Vehicle::ensure_k(L_theta);

	Environment::position xymin(XMIN, YMIN), xymax(XMAX, YMAX);
	Environment::EnvironMap environmap(xymin, xymax);
	Environment::map_construct(&environmap);

	Collision::collision* collimap = new Collision::collision(&environmap);
	
	VectorXf xi(6), xg(5);
	xi << 0.5f, 0.f, 0.f, 0.f, 4.f, 0.f;
	xg << 3.5f, 6.85f, -PI, 0.f, 4.f;
	Vector3f bound;
	bound << xi[4], xi[5], xg[4];
	Trajectory::State XI(xi);
	Trajectory::State XG(xg);

	auto start = steady_clock::now();
	RRT::RTG_RRT rrt(*XI._node(), *XG._node(), 1);
	int result = rrt.search(collimap, L_theta);
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