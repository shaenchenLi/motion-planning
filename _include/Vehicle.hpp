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
		Node(const float &x0, const float &y0, const float &theta0, const float &k0) :x(x0), y(y0), theta(theta0), k(k0) {}
		Node(const float &x0, const float &y0, const float &theta0) :Node(x0, y0, theta0, 0) {}
		Node(const VectorXf &X) :x(X[0]), y(X[1]), theta(X[2]), k(X[3]) {}
		Node(const Node &n) :x(n.x), y(n.y), theta(n.theta), k(n.k) {}
		Node(const Point2D &p) :x(p.x), y(p.y) {}
		Node(const Node *n)
		{
			x = n->x;
			y = n->y;
			theta = n->theta;
			k = n->k;
		}

		inline void reset(const float &x0, const float &y0)
		{
			x = x0;
			y = y0;
		}
		inline void reset(const float &x0, const float &y0, const float &theta0)
		{
			reset(x0, y0);
			reset(theta0);
		}
		inline void reset(const float &theta0)
		{
			theta = theta0;
		}
		inline void reset(const float &x0, const float &y0, const float &theta0, const float &k0)
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

		float x, y, theta, k;
	};

	static bool is_node_effect(const Vehicle::Node &new_node)
	{
		// estimate point A
		float x = new_node.x - DL*cosf(new_node.theta) - W*sinf(new_node.theta) / 2;
		if (x <= XMIN || x >= XMAX)
			return false;
		float y = new_node.y - DL*sinf(new_node.theta) + W*cosf(new_node.theta) / 2;
		if (y <= YMIN || y >= YMAX)
			return false;

		//estimate point B
		x += W*sinf(new_node.theta);
		if (x <= XMIN || x >= XMAX)
			return false;
		y -= W*cosf(new_node.theta);
		if (y <= YMIN || y >= YMAX)
			return false;

		// estimate point C
		x += L*cosf(new_node.theta);
		if (x <= XMIN || x >= XMAX)
			return false;
		y += L*sinf(new_node.theta);
		if (y <= YMIN || y >= YMAX)
			return false;

		// estimate point D
		x -= W*sinf(new_node.theta);
		if (x <= XMIN || x >= XMAX)
			return false;
		y += W*cosf(new_node.theta);
		if (y <= YMIN || y >= YMAX)
			return false;

		return true;
	}

	static bool is_node_effect(const float &x0, const float &y0, const float &theta0)
	{
		// estimate point A
		float x = x0 - DL*cosf(theta0) - W*sinf(theta0) / 2;
		if (x <= XMIN || x >= XMAX)
			return false;
		float y = y0 - DL*sinf(theta0) + W*cosf(theta0) / 2;
		if (y <= YMIN || y >= YMAX)
			return false;

		//estimate point B
		x += W*sinf(theta0);
		if (x <= XMIN || x >= XMAX)
			return false;
		y -= W*cosf(theta0);
		if (y <= YMIN || y >= YMAX)
			return false;

		// estimate point C
		x += L*cosf(theta0);
		if (x <= XMIN || x >= XMAX)
			return false;
		y += L*sinf(theta0);
		if (y <= YMIN || y >= YMAX)
			return false;

		// estimate point D
		x -= W*sinf(theta0);
		if (x <= XMIN || x >= XMAX)
			return false;
		y += W*cosf(theta0);
		if (y <= YMIN || y >= YMAX)
			return false;

		return true;
	}

	static float _L_min(const float &theta, const float err = 0.f)
	{
		return (sinf(theta)*powf((1 - cosf(theta)) / 8, -1.5) / 6.f / std::min(KMAX, std::abs(KMIN)) + err);
	}

	static void ensure_k(vector<float> *L_theta)
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