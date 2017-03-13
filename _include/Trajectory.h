#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <algorithm>
#include <math.h>
#include <vector>
#include <fstream>

#include "tinysplinecpp.h"

#include "Collision_check.h"
#include "Parameters.h"
#include "Vehicle.h"

extern Collision::collision* collimap;

namespace Trajectory
{
	double norm_theta_0_2pi(const double &theta);
	void norm_theta_0_2pi(double *theta);
	void norm_theta_2pi(double *theta); //-pi~pi
	void norm_theta_pi(double *theta);
	double dist(const Vehicle::Node &n1, const Vehicle::Node &n2);
	double dist(const double &x1, const double &y1, const double &x2, const double &y2);
	double dist(const Point2D &p);

	struct State
	{
		State() = default;
		State(const VectorXd &Xi)
		{
			node.x = Xi[0];
			node.y = Xi[1];
			node.theta = Xi[2];
			node.k = Xi[3];
			v = Xi[4];
			s_init = 0;   
			t = 0;
			if (Xi.size() > 5)
				a = Xi[5];
		}
		State(const double &x0, const double &y0, const double &t0, const double &k0) : v(0), s_init(0), a(0), t(0)
		{
			node.x = x0;
			node.y = y0;
			node.theta = t0;
			node.k = k0;
		}
		State(const double &x0, const double &y0, const double &t0, const double &k0, const double &s, const double &v0) : a(0), t(0)
		{
			node.x = x0;
			node.y = y0;
			node.theta = t0;
			node.k = k0;
			s_init = s;
			v = v0;
		}
		State(const double &x0, const double &y0)
		{
			node.x = x0;
			node.y = y0;
		}
		State(const double &x0, const double &y0, const double &t0, const double &k0, const double &s) : v(0), a(0), t(0)
		{
			node.x = x0;
			node.y = y0;
			node.theta = t0;
			node.k = k0;
			s_init = s;
		}
		State(const double &x0, const double &y0, const double &t0, const double &k0, const double &s, const double &v0, const double &a0) :t(0)
		{
			node.x = x0;
			node.y = y0;
			node.theta = t0;
			node.k = k0;
			s_init = s;
			v = v0;
			a = a0;
		}
		State(const double &x0, const double &y0, const double &theta0, const double &k0, const double &s, const double &v0, const double &a0, const double &t0)
		{
			node.x = x0;
			node.y = y0;
			node.theta = theta0;
			node.k = k0;
			s_init = s;
			v = v0;
			a = a0;
			t = t0;
		}
		State(const Vehicle::Node &n, const double &v0, const double &s) :node(n), v(v0), s_init(s), a(0) {}
		State(const State &s) :node(s.node), v(s.v), s_init(s.s_init), a(s.a), t(s.t) {}

		Vehicle::Node* _node() { return &node; }
		double _v() const { return v; }
		double _s_init() const { return s_init; }
		double _theta() const { return node.theta; }
		double _k() const { return node.k; }
		double _a() const { return a; }
		double _t() const { return t; }

		void _node(const double &x0, const double &y0, const double &t0, const double &k0)
		{
			node.x = x0;
			node.y = y0;
			node.theta = t0;
			node.k = k0;
		}
		void _node(const Vehicle::Node &n)
		{
			node.x = n.x;
			node.y = n.y;
			node.theta = n.theta;
			node.k = n.k;
		}
		void _s_init(const double &s) { s_init = s; }
		void  _a(const double &a0) { a = a0; }
		void _v(const double &v0) { v = v0; }
		void _theta(const double &t) { node.theta = t; }
		void _k(const double &k0) { node.k = k0; }
		void _t(const double &t0) { t = t0; }

	private:
		Vehicle::Node node;
		double v, s_init, a, t; //t=dt
	};

	struct traj
	{
		traj()
		{
			bound = new vector<State>(2);
			state_now = new vector<State>;
			state_future = new vector<State>;
			ctrl_points = new vector<Point2D>;
		}
		traj(const VectorXd &Xi, const VectorXd &Xg) 
		{
			bound = new vector<State>;
			state_now = new vector<State>;
			state_future = new vector<State>;
			ctrl_points = new vector<Point2D>;

			bound->emplace_back(Xi);
			bound->emplace_back(Xg);
			seg_begin = *bound->begin();
			seg_end = *bound->end();
			state_now->emplace_back(*bound->begin());
		}
		traj(const State &Xi, const State &Xg, vector<State> *adjust_states_front = nullptr)
		{
			bound = new vector<State>;
			state_now = new vector<State>;
			state_future = new vector<State>;
			ctrl_points = new vector<Point2D>;

			//bound里面存调整过曲率之后的路径的边界值，初始s不为0
			bound->emplace_back(Xi);
			bound->emplace_back(Xg);
			seg_begin = Xi;
			seg_end = Xg;

			if (adjust_states_front != nullptr)
				for (auto &i : *adjust_states_front)
					state_now->push_back(i);
			else
				state_now->push_back(*bound->begin());
		}
		~traj()
		{
			delete state_now;
			delete state_future;
			delete bound;
			delete ctrl_points;
		}
		
		// 根据搜索结果生成路径控制点，之后根据控制点生成B样条曲线
		void _ctrl_points(const vector<Vehicle::Node> &route_tree);
		void _bspline();
		void _bspline(const vector<Vehicle::Node> &route_tree);

		// 根据控制点生成路径点
		void _path();
		// 如果RRT搜索失败，路径两段速度生成，如果RRT搜索成功，根据now中有无State判断路径段的起始速度，然后生成future的路径段
		void _v_init_goal(double *accel, const int &is_search_succeed);
		// 在路径点中加入速度
		bool _v_segments(vector<Point2D> *v_segments); //分段求路径速度
		bool _state(const double &accel);

		// 根据已走路程，输出所需路径段
		void _state(const int &search_result, const double &a0, const vector<Vehicle::Node> &route_tree, vector<State> *adjust_states_end = nullptr); //in practical when sg<state.end-10, calculate state_future

		vector<State>* _state_now() const { return state_now; }
		vector<State>* _state_future() const { return state_future; }
		vector<Point2D>* _ctrl_points() const { return ctrl_points; }

	private:		
		vector<Point2D>* ctrl_points;//可省，调试用
		vector<State>*  state_now;
		vector<State>* state_future;
		ts::BSpline bspline;
		vector<State>* bound;		
		State seg_begin;
		State seg_end;
	};

	void adjust_k(const VectorXd &x, State *state, vector<State> *adjust_states, const int &flag); // flag=1 0-k  flag=-1 k-0
} 

#endif