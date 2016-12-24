#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <algorithm>
#include <math.h>
#include <vector>

#include <Dense>
using namespace Eigen;

#include "tinysplinecpp.h"

#include "Collision_check.h"
#include "Parameters.h"
#include "Vehicle.hpp"

namespace Trajectory
{
	void norm_theta_2pi(double *theta); //-pi~pi
	void norm_theta_pi(double *theta);
	void norm_theta_0_2pi(double *theta);
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
		}
		State(const double &x0, const double &y0, const double &t0, const double &k0)
		{
			node.x = x0;
			node.y = y0;
			node.theta = t0;
			node.k = k0;
		}
		State(const double &x0, const double &y0, const double &t0, const double &k0, const double &s)
		{
			node.x = x0;
			node.y = y0;
			node.theta = t0;
			node.k = k0;
			s_init = s;
		}
		State(const double &x0, const double &y0)
		{
			node.x = x0;
			node.y = y0;
		}
		State(const Vehicle::Node &n, const double &v0, const double &s) :node(n), v(v0), s_init(s) {}
		State(const State &s) :node(s.node), v(s.v), s_init(s.s_init) {}

		Vehicle::Node* _node() { return &node; }
		double _v() const { return v; }
		double _s_init() const { return s_init; }
		double _theta() const { return node.theta; }
		double _k() const { return node.k; }

		void _v(const double &v0) { v = v0; }
		void _theta(const double &t) { node.theta = t; }
		void _k(const double &k0) { node.k = k0; }
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

	private:
		Vehicle::Node node;
		double v, s_init;
	};

	struct traj
	{
		traj()
		{
 			knots = { 1., 0.7, 0.7, 0.4, 0.4, 0.1, 0.1, 0. };
		}
		traj(const VectorXd &Xi, const VectorXd &Xg)
		{
			ctrl_points.emplace_back(Xi[0], Xi[1]);
			knots = { 1., 0. };
			bound.emplace_back(Xi);
			bound.emplace_back(Xg);
			v_tmp_dest.emplace_back(Xg[4]);
			_v_tmp_dest(Xi[5]);
		}
		traj(State Xi, State Xg, const double &accel, vector<State> *adjust_states_front)
		{
			ctrl_points.emplace_back(Xi._node()->x, Xi._node()->y);
			knots = { 1., 0. };
			bound.emplace_back(Xi);
			bound.emplace_back(Xg);
			v_tmp_dest.emplace_back(Xg._v());
			_v_tmp_dest(accel);
			state_now = *adjust_states_front;
		}
		
		void _ctrl_points(const vector<Vehicle::Node> &route_tree, vector<double> *L_theta, Collision::collision *collimap);
		void _bspline();
		void _bspline(const vector<Vehicle::Node> &route_tree);
		void _state(double *accel, vector<State> *adjust_states_end= nullptr);
		vector<State>* _state(const double &a0, const double &sg, vector<State> *adjust_states_end); //in practical when sg<state.end-10, calculate state_future
		void _v_tmp_dest(const double &accel);

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		vector<State>* _state_now() { return &state_now; }
		vector<State>* _state_future() { return &state_future; }
		vector<Point2D>* _ctrl_points() { return &ctrl_points; }

	private:		
		vector<Point2D> ctrl_points;
		vector<double> v_tmp_dest;
		vector<State> state_now;
		vector<State> state_future;
		vector<double> knots; 
		ts::BSpline bspline;
		vector<State> bound;		
	};

	void adjust_k(const VectorXd &x, State *state, vector<State> *adjust_states, const int &flag); // flag=1 0-k  flag=-1 k-0
} 

#endif