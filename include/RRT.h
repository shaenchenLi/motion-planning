#ifndef RRT_H
#define RRT_H

#include <algorithm>
#include <ctime>
//#include <fstream>
//#include <iostream>
#include <list>
#include <math.h>
#include <map>
#include <random>
#include <set>
//#include <stdio.h>
#include <vector>

#include "Collision_check.h"
#include "Force_Extend.h"
#include "Trajectory.h"
#include "Vehicle.h"

using Vector5d = Matrix<double, 5, 1>;
using Vector6d = Matrix<double, 6, 1>;

namespace RRT
{
	struct RRT_Node;

	using Index = vector<RRT_Node>::size_type;
	using DIS_TO_NODE = std::map<double, Index>;

	const int dim = 2;

	inline double norm(const double &raw, const double &max, const double &min) { return std::fabs((max - raw) / (max - min)); }

	struct RRT_Node
	{
		RRT_Node() = default;
		RRT_Node(const RRT_Node &n) :
			node(n.node), predecessor(n.predecessor),
			left(n.left), right(n.right), kd_father(n.kd_father), deep(n.deep), index(n.index) 
		{
			successor = new vector<Index>(*n.successor);
		}
		RRT_Node(RRT_Node *n) :
			node(n->node), predecessor(n->predecessor), 
			left(n->left), right(n->right), kd_father(n->kd_father), deep(n->deep), index(n->index) 
		{
			successor = new vector<Index>(*n->successor);
		}
		RRT_Node(const Vehicle::Node &n, const Index &i, const Index &p) :node(n), index(i), predecessor(p)
		{
			left = right = kd_father = -1; 
			deep = 0;
			successor = new vector<Index>;
		}
		RRT_Node(const Vehicle::Node &n) :node(n) 
		{
			successor = new vector<Index>;
		}
		RRT_Node(Trajectory::State s, const Index &i, const Index &p) : index(i), predecessor(p)
		{
			node = Vehicle::Node(s._node());
			left = right = kd_father = -1;
			deep = 0;
			successor = new vector<Index>;
		}

		~RRT_Node()
		{
			//delete successor;
		}

		void reset_node(double x, double y)
		{
			node.x = x;
			node.y = y;
		}
		void reset_node(double x, double y, double theta) { reset_node(x, y); node.theta = theta; }
		double edge_generate(const Vehicle::Node &node_i, const Vehicle::Node &node_g);
		void reset(const RRT_Node &r)
		{
			node.reset(r.node);
			index = r.index;
			predecessor = r.predecessor;
			left = r.left;
			right = r.right;
			kd_father = r.kd_father;
			deep = r.deep;
			if (successor->size() > 0)
				vector<Index>().swap(*successor);
			//cout << "successor has been swapped" << endl;
			if (!r.successor->empty())
			{
				for (auto i : *r.successor)
					successor->emplace_back(i);
			}
		}
		void reset(const Vehicle::Node &n)
		{
			node.reset(n);
			index = predecessor = left = right = kd_father = -1;
			deep = 0;
			successor = new vector<Index>;
		}

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
		RTG_RRT(const Vehicle::Node &xi, const Vehicle::Node &xg, const int &f) :flag(f), max_iter(MAXITER_1)
		{
			if (xg.x - xi.x > RRTSTEP_l || xg.y - xi.y > RRTSTEP_l)
				step = RRTSTEP_l;
			else
				step = RRTSTEP_s;
			//root = new RRT_Node(xi, 0, 0);
			dest = new RRT_Node(xg, -1, -1);
			tree = new vector<RRT_Node>({ RRT_Node(xi, 0, 0) });
			route_tree = new vector<Vehicle::Node>;
			//tree->reserve(max_size + 1);
			//tree->emplace_back(root);
			//motion = new vector<Trajectory::State>;
		}
		RTG_RRT(Trajectory::State state_i, Trajectory::State state_g, const int f = 1) :flag(f), max_iter(MAXITER_1)
		{
			if (state_g._node()->x - state_i._node()->x > RRTSTEP_l || state_g._node()->y - state_i._node()->y > RRTSTEP_l)
				step = RRTSTEP_l;
			else
				step = RRTSTEP_s;
			//root = new RRT_Node(state_i, 0, 0);
			dest = new RRT_Node(state_g, -1, -1);
			tree = new vector<RRT_Node>({ RRT_Node(state_i, 0, 0) });
			route_tree = new vector<Vehicle::Node>;
			//tree->reserve(max_size + 1);
			//tree->emplace_back(root);
			//motion = new vector<Trajectory::State>;
		}
		RTG_RRT(const RTG_RRT &r) :step(r.step), flag(r.flag), max_iter(r.max_iter)
		{
			//root = new RRT_Node(*r._root());
			dest = new RRT_Node(*r._dest());
			//motion = new vector<Trajectory::State>(*r._motion());
			tree = new vector<RRT_Node>(*r._tree());
			route_tree = new vector<Vehicle::Node>(*r._route_tree());
		}
		RTG_RRT() :RTG_RRT(Vehicle::Node(), Vehicle::Node(), 0) {}
		~RTG_RRT()
		{
			//cout << root->_node()->x << " " << root->_node()->y << endl;
			//cout << dest->_node()->x << " " << dest->_node()->y << endl;			
			delete dest;
			//delete motion;
		}

		// NN_search
		void kd_insert(RRT_Node *new_node, const Index &insert_index); //insert
		void kNN_search(const Vehicle::Node &node, DIS_TO_NODE *kNN);
		int kNN_add(const Vehicle::Node &node, const Index &add_tmp, DIS_TO_NODE *kNN);//1：插入成功 2：插入但未成功 3：不能插入
		void nearest_search(const Vehicle::Node &node, Index* near_node);  //virtual
		void near_search(const Vehicle::Node &node, vector<Index>* near_node);

		// random select
		void _lambda_ss(double *lambda, const int &num, const double le = 0.); //le=0:unextendable  le!=0:extendable
		void rand_normal(const int &iter, Vehicle::Node *rand_node);
		void rand_gaussian(const int &iter, vector<double> *gaus_para, position<double> *refer_point, Vehicle::Node *rand_node);
		void gaussian_para(const int &type, vector<double> *gaus_para);
		void gaussian_para_turn(const double &l, const int &i, vector<double> *gaus_para, position<double> *refer_point);
		void gaussian_para_Uturn(const double &l, const int &i, vector<double> *gaus_para, position<double> *refer_point);

		// template path generate
		int force_extend(const int &type, double *le, const double l_v_theta = 0.);//2:success 1:partially extension 0:failure
		int force_extend(const int &type, const vector<double> &c, double *le, const double l_v_theta = 0.);
		void path2tree(const Index &predecessor, vector<Vehicle::Node> *path);

		// grow of tree
		int grow(Vehicle::Node *new_node); //2:success 1:partially extension 0:failure
		int search(const int &type, const vector<double> constraint = { 0 }, const double l_v_theta = 0.);
		
		// get final path 
		void getpath();
		void getpath(const Index &end);

		//void _motion(const vector<Trajectory::State> &t);
		void reset(const Index &new_root);
		void reset(const Trajectory::State &s);
		void reset(Trajectory::State state_i, Trajectory::State state_g, const int f = 1);
		void _copy(const RTG_RRT &copied_tree);
		
		vector<RRT_Node>* _tree() const { return tree; }
		vector<Vehicle::Node>* _route_tree() const { return route_tree; }
		RRT_Node* _dest() const { return dest; }
		//vector<Trajectory::State>* _motion() const { return motion; }

		int max_iter;

	private:
		Vehicle::Node root_global;
		RRT_Node *dest;
		vector<RRT_Node>* tree;
		vector<Vehicle::Node>* route_tree;
		double step;
		int flag; //flag=0:basic RRT; flag=1:RTG_RRT  真正使用时可省略
		//vector<Trajectory::State> *motion;//仿真画图方便，实际不用
	};
}

#endif