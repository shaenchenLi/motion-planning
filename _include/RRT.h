#ifndef RRT_H
#define RRT_H

#include <algorithm>
#include <ctime>
#include <list>
#include <math.h>
#include <random>
#include <vector>

#include <Dense>
using namespace Eigen;
using Vector5f = Matrix<float, 5, 1>;
using Vector6f = Matrix<float, 6, 1>;

#include "Collision_check.h"
#include "Force_Extend.h"
#include "Trajectory.h"
#include "Vehicle.hpp"

namespace RRT
{
	struct RRT_Node;

	using Index = vector<RRT_Node>::size_type;
	using DIS_TO_NODE = std::map<float, Index>;

	const int dim = 2;

	inline float norm(const float &raw, const float &max, const float &min) { return std::fabs((max - raw) / (max - min)); }

	struct RRT_Node
	{
		RRT_Node() = default;
		RRT_Node(const RRT_Node &n) :
			node(n.node), predecessor(n.predecessor), successor(n.successor),
			left(n.left), right(n.right), kd_father(n.kd_father), deep(n.deep), index(n.index) {}
		RRT_Node(RRT_Node *n) :
			node(n->node), predecessor(n->predecessor), successor(n->successor),
			left(n->left), right(n->right), kd_father(n->kd_father), deep(n->deep), index(n->index) {}
		RRT_Node(const Vehicle::Node &n, const Index &i, const Index &p, const Index &max) :node(n), index(i), predecessor(p)
		{
			left = right = kd_father = max; //max equals to the max_size of tree +1
			deep = 0;
			successor = new vector<Index>;
		}
		RRT_Node(const Vehicle::Node &n) :node(n) 
		{
			successor = new vector<Index>;
		}

		~RRT_Node()
		{
			delete successor;
		}

		void reset_node(float x, float y)
		{
			node.x = x;
			node.y = y;
		}
		void reset_node(float x, float y, float theta) { reset_node(x, y); node.theta = theta; }
		float edge_generate(const Vehicle::Node &node_i, const Vehicle::Node &node_g);

		//getdata
		Vehicle::Node* _node() { return &node; }
		Index _predecessor() const { return predecessor; }
		Index _left() const { return left; }
		Index _right() const { return right; }
		Index _kd_father() const { return right; }
		int _deep() const { return deep; }
		Index _index() { return index; }

		//setdata
		void _index(const Index i) { index = i; }
		void _left(const Index &l) { left = l; }
		void _right(const Index &r) { right = r; }
		void _kd_father(const Index &f) { kd_father = f; }
		void _predecessor(const Index &p) { predecessor = p; }
		void _successor(const Index &s) { successor->emplace_back(s); }
		void _deep(const int &d) { deep = d % dim; }

	private:
		Vehicle::Node node;
		Index index;
		Index predecessor;
		vector<Index>* successor;
		Index left, right, kd_father;
		int deep;
	};

	struct RTG_RRT
	{
		RTG_RRT(const Vehicle::Node &xi, const Vehicle::Node &xg, const int &f) :flag(f), max_size(MAXITER)
		{
			if (xg.x - xi.x > RRTSTEP_l || xg.y - xi.y > RRTSTEP_s)
				step = RRTSTEP_l;
			else
				step = RRTSTEP_s;
			root = new RRT_Node(xi, 0, 0, max_size + 1);
			dest = new RRT_Node(xg, max_size + 1, max_size + 1, max_size + 1);
			tree = new vector<RRT_Node>;
			route_tree = new vector<Vehicle::Node>;
			tree->reserve(max_size + 1);
			tree->emplace_back(root);
		}
		RTG_RRT() :RTG_RRT(Vehicle::Node(), Vehicle::Node(), 0) {}
		~RTG_RRT()
		{
			delete root, dest;
			delete tree, route_tree;
		}

		// NN_search
		void kd_insert(RRT_Node *new_node, const Index &insert_index); //insert
		void kNN_search(const Vehicle::Node &node, DIS_TO_NODE *kNN);
		int kNN_add(const Vehicle::Node &node, const Index &add_tmp, DIS_TO_NODE *kNN);//1：插入成功 2：插入但未成功 3：不能插入
		void nearest_search(const Vehicle::Node &node, Index* near_node);  //virtual

		// random select
		void rand_select(Vehicle::Node *rand_node, const int &iter); //virtual
		void _lambda_ss(float *lambda, const int &num, const float le = 0.f); //le=0:unextendable  le!=0:extendable

		// template path generate
		int force_extend(vector<float> *L_theta, Collision::collision *collimap, float *le);//2:success 1:partially extension 0:failure
		int force_extend(const float &l, const float &w, const float &r, vector<float> *L_theta, Collision::collision *collimap, float *le);
		void path2tree(const Index &predecessor, vector<Vehicle::Node> *path);

		// grow of tree
		int grow(Vehicle::Node *new_node, Collision::collision *collimap); //2:success 1:partially extension 0:failure
		int search(Collision::collision *collimap, vector<float> *L_theta, const float l = 0, const float w = 0, const float r = 0); //virtual

		// get final path 
		void getpath();

		vector<RRT_Node>* _tree() { return tree; }
		vector<Vehicle::Node>* _route_tree() { return route_tree; }

	private:
		RRT_Node *root, *dest;
		vector<RRT_Node>* tree;
		vector<Vehicle::Node>* route_tree;
		float step;
		int max_size;
		int flag; //flag=0:basic RRT; flag=1:TG_RRTG_RRTRT
	};
}

#endif