#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <algorithm>
#include <math.h>
#include <map>
#include <vector>
#include <fstream>

#include "Configuration.h"
#include "Collision_check.h"
#include "Vehicle.h"
using namespace Collision;

namespace Trajectory
{
	double norm_theta_0_2pi(const double &theta);
	void norm_theta_0_2pi(double *theta);
	void norm_theta_2pi(double *theta); //-pi~pi
	void norm_theta_pi(double *theta);
	double dist(const Vehicle::Node &n1, const Vehicle::Node &n2);
	double dist(const double &x1, const double &y1, const double &x2, const double &y2);
	double dist(const position<double> &p);
	double dist(const position<double> &p1, const position<double> &p2);

	struct State
	{
		State()
		{
			node.x = node.y = node.theta = node.k = v = s_init = a = 0.;
		}

		State(const VectorXd &Xi)
		{
			node.x = Xi[0];
			node.y = Xi[1];
			node.theta = Xi[2];
			node.k = Xi[3];
			v = Xi[4];
			s_init = 0;   
			//t = 0;
			if (Xi.size() > 5)
				a = Xi[5];
		}
		State(const double &x0, const double &y0, const double &t0, const double &k0) : v(0), s_init(0), a(0)//, t(0)
		{
			node.x = x0;
			node.y = y0;
			node.theta = t0;
			node.k = k0;
		}
		State(const double &x0, const double &y0, const double &t0, const double &k0, const double &s, const double &v0) : a(0)//, t(0)
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
		State(const double &x0, const double &y0, const double &t0, const double &k0, const double &s) : v(0), a(0)//, t(0)
		{
			node.x = x0;
			node.y = y0;
			node.theta = t0;
			node.k = k0;
			s_init = s;
		}
		State(const double &x0, const double &y0, const double &t0, const double &k0, const double &s, const double &v0, const double &a0)// :t(0)
		{
			node.x = x0;
			node.y = y0;
			node.theta = t0;
			node.k = k0;
			s_init = s;
			v = v0;
			a = a0;
		}
/*		State(const double &x0, const double &y0, const double &theta0, const double &k0, const double &s, const double &v0, const double &a0)//, const double &t0)
		{
			node.x = x0;
			node.y = y0;
			node.theta = theta0;
			node.k = k0;
			s_init = s;
			v = v0;
			a = a0;
			//t = t0;
		}*/
		State(const Vehicle::Node &n, const double &v0, const double &s) :node(n), v(v0), s_init(s), a(0) {}
		State(const State &s) :node(s.node), v(s.v), s_init(s.s_init), a(s.a)/*, t(s.t) */{}
		~State() {}

		bool operator==(const State &s) const
		{
			double theta0 = std::fmod(node.theta, TWO_PI);
			if (theta0 < 0)
				theta0 += 2 * PI;
			double theta1 = std::fmod(s.node.theta, TWO_PI);
			if (theta1 < 0)
				theta1 += 2 * PI;
			if ((node.x == s.node.x) && (node.y == s.node.y) && (theta0 - theta1 < 10e-6) && (node.k - s.node.k < 10e-8))
			{
				if (v == s.v&&a == s.a)
					return true;
				else
					return false;
			}
			else
				return false;
		}

		void reset()
		{
			node.reset();
			v = a = /*t = */s_init = 0;
		}

		Vehicle::Node* _node() { return &node; }
		double _v() const { return v; }
		double _s() const { return s_init; }
		double _theta() const { return node.theta; }
		double _k() const { return node.k; }
		double _a() const { return a; }
		//double _t() const { return t; }

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
		void _s(const double &s) { s_init = s; }
		void  _a(const double &a0) { a = a0; }
		void _v(const double &v0) { v = v0; }
		void _theta(const double &t) { node.theta = t; }
		void _k(const double &k0) { node.k = k0; }
		//void _t(const double &t0) { t = t0; }

	private:
		Vehicle::Node node;
		double v, s_init, a;// , t; //t=dt
	};

	struct traj
	{
		traj()
		{
			bound = new vector<State>(2);
			//state_now = new vector<State>;
			entire_traj = new vector<State>;
			ctrl_points = new vector<position<double>>;
			u_div = new vector<int>;
			v_div = new vector<double>;
		}
		traj(const VectorXd &Xi, const VectorXd &Xg) 
		{
			bound = new vector<State>;
			//state_now = new vector<State>;
			entire_traj = new vector<State>;
			ctrl_points = new vector<position<double>>;
			u_div = new vector<int>;
			v_div = new vector<double>;

			bound->emplace_back(Xi);
			bound->emplace_back(Xg);
			seg_begin = *bound->begin();
			seg_end = *bound->end();
			entire_traj->emplace_back(*bound->begin());
		}
		traj(const State &Xi, const State &Xg, vector<State> *adjust_states_front = nullptr)
		{
			bound = new vector<State>;
			//state_now = new vector<State>;
			entire_traj = new vector<State>;
			ctrl_points = new vector<position<double>>;
			u_div = new vector<int>;
			v_div = new vector<double>;

			//bound里面存调整过曲率之后的路径的边界值，初始s不为0
			bound->emplace_back(Xi);
			bound->emplace_back(Xg);
			seg_begin = Xi;
			seg_end = Xg;

			if (adjust_states_front != nullptr)
				for (auto &i : *adjust_states_front)
					entire_traj->push_back(i);
			else
				entire_traj->push_back(*bound->begin());
		}
		~traj()
		{
			//delete state_now;
			delete entire_traj;
			delete bound;
			delete ctrl_points;
			delete u_div;
			delete v_div;
		}
		
		void reset(const State &Xi, const State &Xg);
		
		// 根据搜索结果生成路径控制点，之后根据控制点生成B样条曲线
		void _ctrl_points(const vector<Vehicle::Node> &route_tree);
		void _bspline();
		void _bspline(const vector<Vehicle::Node> &route_tree);

		// 根据控制点生成路径点
		bool _path();
		// 如果RRT搜索失败，路径两段速度生成，如果RRT搜索成功，根据now中有无State判断路径段的起始速度，然后生成future的路径段
		void _v_init_goal(const int &is_search_succeed);
		// 在路径点中加入速度
		bool _v_segments(const int u_base = 0); //分段求路径速度

		//根据RRT搜索结果生成路径
		bool _path(const int &search_result, const vector<Vehicle::Node> &route_tree);
		
		//vector<State>* _state_now() const { return state_now; }
		vector<State>* _entire_traj() const { return entire_traj; }
		vector<position<double>>* _ctrl_points() const { return ctrl_points; }
		State _seg_end() const { return seg_end; }
		State _seg_begin() const { return seg_begin; }
		vector<State>* _bound() const { return bound; }
		vector<double>* _v_div() const { return v_div; }
		vector<int>* _u_div() const { return u_div; }

	private:		
		vector<position<double>>* ctrl_points;//可省，调试用
		//vector<State>*  state_now;
		vector<State>* entire_traj;
		ts::BSpline bspline;
		vector<State>* bound;		
		State seg_begin;
		State seg_end;
		vector<int>* u_div;
		vector<double>* v_div;
	};
} 

#endif