#ifndef VEHICLE_H
#define VEHICLE_H

//#include <algorithm>
#include <vector>
#include <iostream>
using std::cout; using std::endl;
using std::vector;

#include <Dense>
using namespace Eigen;

#include "Configuration.h"

extern vector<double> L_theta;

template<typename T> struct position
{
	T x, y;
	position() :x(0), y(0) {}
	position(const T &a, const T &b) :x(a), y(b) {}

	void reset(const T &x0, const T &y0)
	{
		x = x0;
		y = y0;
	}
};

template<typename T> struct time_position
{
	T x, y, t;
	time_position(const T &x0, const T &y0, const T &t0) : x(x0), y(y0), t(t0) {}
};

namespace Vehicle
{
	struct Node
	{
		Node() = default;
		Node(const double &x0, const double &y0, const double &theta0, const double &k0) :x(x0), y(y0), theta(theta0), k(k0) {}
		Node(const double &x0, const double &y0, const double &theta0) :Node(x0, y0, theta0, 0) {}
		Node(const VectorXd &X) :x(X[0]), y(X[1]), theta(X[2]), k(X[3]) {}
		Node(const Node &n) :x(n.x), y(n.y), theta(n.theta), k(n.k) {}
		Node(const Node *n)
		{
			x = n->x;
			y = n->y;
			theta = n->theta;
			k = n->k;
		}

		inline void reset()
		{
			x = y = theta = k = 0;
		}

		inline void reset(const double &x0, const double &y0)
		{
			x = x0;
			y = y0;
		}
		inline void reset(const double &x0, const double &y0, const double &theta0)
		{
			reset(x0, y0);
			reset(theta0);
		}
		inline void reset(const double &theta0)
		{
			theta = theta0;
		}
		inline void reset(const double &x0, const double &y0, const double &theta0, const double &k0)
		{
			reset(x0, y0, theta0);
			k = k0;
		}
		inline void reset(const Node &n)
		{
			x = n.x;
			y = n.y;
			theta = n.theta;
			k = n.k;
		}
		bool operator==(const Node &n)
		{
			double theta0 = std::fmod(theta, TWO_PI);
			if (theta0 < 0)
				theta0 += 2 * PI;
			double theta1 = std::fmod(n.theta, TWO_PI);
			if (theta1 < 0)
				theta1 += 2 * PI;
			return (x == n.x) && (y == n.y) && (theta0 - theta1 < 10e-6) && (k - n.k < 10e-8);
		}
		bool operator!=(const Node &n)
		{
			return !(*this == n);
		}

		double x, y, theta, k;
	};
}

extern Vehicle::Node map_origin;

namespace Vehicle
{
	bool is_node_effect(const Vehicle::Node &new_node);
	bool is_node_effect(const double &x0, const double &y0, const double &theta0);

	inline double _L_min(const double &theta, const double err = ERROR_KMAX)
	{
		double  new_theta = std::fmod(theta, TWO_PI);
		if (new_theta < 0)
			new_theta += TWO_PI;
		if (new_theta > PI)
			new_theta = 2 * PI - new_theta;

		return (sin(new_theta)*pow((1 - cos(new_theta)) / 8, -1.5) / 6. / KMAX + err);
	}

	vector<double> ensure_k();


	void orient_trans_global_vehicle(const Vehicle::Node &xi, Vehicle::Node *xg);
	void orient_trans_global_vehicle(const Vehicle::Node &xi, vector<double> *control);
	void orient_trans_vehicle_global(const Vehicle::Node &xi, Vehicle::Node *xg);
	void orient_trans_vehicle_global(const Vehicle::Node &xi, vector<Vehicle::Node> *xg);
	void orient_trans_vehicle_global(const Vehicle::Node &xi, vector<double> *control);
}

#endif