#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <iostream>
using std::cout; using std::endl;
using std::vector;

#include <Dense>
using namespace Eigen;

#include "Parameters.h"

extern vector<double> L_theta;

namespace Vehicle
{
	struct Node
	{
		Node() = default;
		Node(const double &x0, const double &y0, const double &theta0, const double &k0) :x(x0), y(y0), theta(theta0), k(k0) {}
		Node(const double &x0, const double &y0, const double &theta0) :Node(x0, y0, theta0, 0) {}
		Node(const VectorXd &X) :x(X[0]), y(X[1]), theta(X[2]), k(X[3]) {}
		Node(const Node &n) :x(n.x), y(n.y), theta(n.theta), k(n.k) {}
		Node(const Point2D &p) :x(p.x), y(p.y) {}
		Node(const Node *n)
		{
			x = n->x;
			y = n->y;
			theta = n->theta;
			k = n->k;
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
		bool operator==(Node &n)
		{
			double theta0 = std::fmod(theta, TWO_PI);
			if (theta0 < 0)
				theta0 += 2 * PI;
			double theta1 = std::fmod(n.theta, TWO_PI);
			if (theta1 < 0)
				theta1 += 2 * PI;
			return (x == n.x) && (y == n.y) && (theta0 - theta1 < 10e-6) && (k - n.k < 10e-8);
		}
		bool operator!=(Node &n)
		{
			return !(*this == n);
		}

		double x, y, theta, k;
	};

	bool is_node_effect(const Vehicle::Node &new_node);
	bool is_node_effect(const double &x0, const double &y0, const double &theta0);

	inline double _L_min(const double &theta, const double err = 0.05)
	{
		return (sin(theta)*pow((1 - cos(theta)) / 8, -1.5) / 6. / std::min(KMAX, std::abs(KMIN)) + err);
	}

	vector<double> ensure_k();
}

#endif