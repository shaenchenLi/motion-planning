#ifndef VEHICLE_H
#define VEHICLE_H

#include <Dense>
using namespace Eigen;

#include "Parameters.h"

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
		bool operator==(Node &n)
		{
			return (x == n.x) && (y == n.y) && (theta - n.theta < 10e-6) && (k - n.k < 10e-8);
		}
		bool operator!=(Node &n)
		{
			return !(*this == n);
		}

		double x, y, theta, k;
	};

	static bool is_node_effect(const Vehicle::Node &new_node)
	{
		// estimate point A
		double x = new_node.x - DL*cos(new_node.theta) - W*sin(new_node.theta) / 2;
		if (x <= XMIN || x >= XMAX)
			return false;
		double y = new_node.y - DL*sin(new_node.theta) + W*cos(new_node.theta) / 2;
		if (y <= YMIN || y >= YMAX)
			return false;

		//estimate point B
		x += W*sin(new_node.theta);
		if (x <= XMIN || x >= XMAX)
			return false;
		y -= W*cos(new_node.theta);
		if (y <= YMIN || y >= YMAX)
			return false;

		// estimate point C
		x += L*cos(new_node.theta);
		if (x <= XMIN || x >= XMAX)
			return false;
		y += L*sin(new_node.theta);
		if (y <= YMIN || y >= YMAX)
			return false;

		// estimate point D
		x -= W*sin(new_node.theta);
		if (x <= XMIN || x >= XMAX)
			return false;
		y += W*cos(new_node.theta);
		if (y <= YMIN || y >= YMAX)
			return false;

		return true;
	}

	static bool is_node_effect(const double &x0, const double &y0, const double &theta0)
	{
		// estimate point A
		double x = x0 - DL*cos(theta0) - W*sin(theta0) / 2;
		if (x <= XMIN || x >= XMAX)
			return false;
		double y = y0 - DL*sin(theta0) + W*cos(theta0) / 2;
		if (y <= YMIN || y >= YMAX)
			return false;

		//estimate point B
		x += W*sin(theta0);
		if (x <= XMIN || x >= XMAX)
			return false;
		y -= W*cos(theta0);
		if (y <= YMIN || y >= YMAX)
			return false;

		// estimate point C
		x += L*cos(theta0);
		if (x <= XMIN || x >= XMAX)
			return false;
		y += L*sin(theta0);
		if (y <= YMIN || y >= YMAX)
			return false;

		// estimate point D
		x -= W*sin(theta0);
		if (x <= XMIN || x >= XMAX)
			return false;
		y += W*cos(theta0);
		if (y <= YMIN || y >= YMAX)
			return false;

		return true;
	}

	static double _L_min(const double &theta, const double err = 0.f)
	{
		return (sin(theta)*pow((1 - cos(theta)) / 8, -1.5) / 6.f / std::min(KMAX, std::abs(KMIN)) + err);
	}

	static void ensure_k(vector<double> *L_theta)
	{
		L_theta->push_back(8 * PI / 9);
		L_theta->push_back(_L_min(*L_theta->rbegin()));
		for (int i = 0; i < 6; i++) 
		{
			L_theta->push_back(5 * PI / 6 - i*0.1f*PI);
			L_theta->push_back(_L_min(*L_theta->rbegin()));
		}
	}
}

#endif