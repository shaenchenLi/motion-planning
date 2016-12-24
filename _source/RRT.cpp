#include "RRT.h"

void RRT::RTG_RRT::kd_insert(RRT_Node *new_node, const Index &insert_index)
{
	Index curr = 0;
	int flag = 0;

	while (flag == 0)
	{
		switch ((*tree)[curr]._deep())
		{
		case 0:
			if (norm(new_node->_node()->x, XMAX, XMIN) > norm((*tree)[curr]._node()->x, XMAX, XMIN))
			{
				if ((*tree)[curr]._right() != max_size + 1)
					curr = (*tree)[curr]._right();
				else
				{
					(*tree)[curr]._right(new_node->_index());
					flag = 1;
				}
			}
			else
			{
				if ((*tree)[curr]._left() != max_size + 1)
					curr = (*tree)[curr]._left();
				else
				{
					(*tree)[curr]._left(new_node->_index());
					flag = 1;
				}
			}
			break;
		case 1:
			if (norm(new_node->_node()->y, dest->_node()->y, 0) > norm((*tree)[insert_index]._node()->y, dest->_node()->y, 0))
			{
				if ((*tree)[curr]._right() != max_size + 1)
					curr = (*tree)[curr]._right();
				else
				{
					(*tree)[curr]._right(new_node->_index());
					flag = 1;
				}
			}
			else
			{
				if ((*tree)[curr]._left() != max_size + 1)
					curr = (*tree)[curr]._left();
				else
				{
					(*tree)[curr]._left(new_node->_index());
					flag = 1;
				}
			}
			break;
		}
	}

	new_node->_kd_father(curr);
	new_node->_deep((*tree)[curr]._deep() + 1);
}

int RRT::RTG_RRT::kNN_add(const Vehicle::Node &node, const Index &add_tmp, DIS_TO_NODE *kNN)
{
	bool result;
	if (kNN->size() < kNN_NUM)
		result = kNN->insert({ Trajectory::dist(node, *(*tree)[add_tmp]._node()), add_tmp }).second;
	else
	{
		double dist_tmp = Trajectory::dist(node, *(*tree)[add_tmp]._node());
		if (dist_tmp < kNN->rbegin()->second)
		{
			kNN->erase(kNN->rbegin()->first);
			result = kNN->insert({ dist_tmp, add_tmp }).second;
		}
		else
			return -1;
	}
	if (result == true)
		return 1;
	else
		return 0;
}

void RRT::RTG_RRT::kNN_search(const Vehicle::Node &node, DIS_TO_NODE *kNN)
{
	DIS_TO_NODE *BPQ = new DIS_TO_NODE;
	std::set<Index> *BPQ_old = new std::set<Index>;

	Index curr = 0;
	int bt_num = 0;
	while (1)
	{
		Index old = 0;
		while (curr != max_size + 1)
		{
			old = curr;
			switch ((*tree)[curr]._deep())
			{
			case 0:
				if (norm(node.x, XMAX, XMIN) > norm((*tree)[curr]._node()->x, XMAX, XMIN))
				{
					old = curr;
					curr = (*tree)[curr]._right();
				}
				else
				{
					old = curr;
					curr = (*tree)[curr]._left();
				}
				break;
			case 1:
				if (norm(node.y, dest->_node()->y, 0) > norm((*tree)[curr]._node()->y, dest->_node()->y, 0))
				{
					old = curr;
					curr = (*tree)[curr]._right();
				}
				else
				{
					old = curr;
					curr = (*tree)[curr]._left();
				}
				break;
			}

			if (BPQ_old->find(old) == BPQ_old->end())
				BPQ->insert({ Trajectory::dist(node, *(*tree)[old]._node()), old });//避免重复插入BPQ
		}
		curr = old;	//正向搜索最邻近点		

		double dis_spilt;
		switch (kNN_add(node, curr, kNN))
		{
		case 1: case -1:
			if ((*tree)[curr]._deep() == 0)
				dis_spilt = abs((*tree)[curr]._node()->x - node.x);
			else
				dis_spilt = abs((*tree)[curr]._node()->y - node.y);
			break;
		case 0:
			Index spilt_node = kNN->find(Trajectory::dist(node, *(*tree)[curr]._node()))->second;
			if (spilt_node == 0)
				dis_spilt = abs((*tree)[spilt_node]._node()->x - node.x);
			else
				dis_spilt = abs((*tree)[spilt_node]._node()->y - node.y);
			break;
		}
		if (kNN->rbegin()->first > dis_spilt || kNN->size() < kNN_NUM)
		{
			if (bt_num < BT_MAX)
			{
				while (!BPQ->empty())
				{
					curr = BPQ->begin()->second;
					Index tmp = curr;
					BPQ->erase(BPQ->begin());
					BPQ_old->insert(curr);

					switch ((*tree)[curr]._deep()) //判断另一侧是否有结点从而判断是否需要回溯
					{
					case 0:
						if (norm(node.x, XMAX, XMIN) < norm((*tree)[curr]._node()->x, XMAX, XMIN))
						{
							curr = (*tree)[curr]._right();
							if (curr == max_size + 1)
							{
								curr = tmp;
								kNN_add(node, curr, kNN);
								break;
							}
							else
								kNN_add(node, tmp, kNN);
						}
						else
						{
							curr = (*tree)[curr]._left();
							if (curr == max_size + 1)
							{
								curr = tmp;
								kNN_add(node, curr, kNN);
								break;
							}
							else
								kNN_add(node, tmp, kNN);
						}
						break;
					case 1:
						if (norm(node.y, dest->_node()->y, 0) < norm((*tree)[curr]._node()->y, dest->_node()->y, 0))
						{
							curr = (*tree)[curr]._right();
							if (curr == max_size + 1)
							{
								curr = tmp;
								kNN_add(node, curr, kNN);
								break;
							}
							else
								kNN_add(node, tmp, kNN);
						}
						else
						{
							curr = (*tree)[curr]._left();
							if (curr == max_size + 1)
							{
								curr = tmp;
								kNN_add(node, curr, kNN);
								break;
							}
							else
								kNN_add(node, tmp, kNN);
						}
						break;
					}

					if (tmp == curr)
						continue;
					else
						break;
				}

				if (BPQ->empty() && (curr == 0 || curr == old))
					break;
			}
			else
				break;
		}
	}
	delete BPQ, BPQ_old;
}

void RRT::RTG_RRT::nearest_search(const Vehicle::Node &node, Index* near_node)
{
	DIS_TO_NODE *kNN = new DIS_TO_NODE;
	kNN_search(node, kNN);

	Index near_node_tmp;
	double min_dist = 1000;
	for (auto &i : *kNN)
	{
		double dist = i.first;
		if (dist < min_dist)
		{
			min_dist = dist;
			near_node_tmp = i.second;
		}
	}
	*near_node = near_node_tmp;
}

void RRT::RTG_RRT::rand_select(Vehicle::Node *rand_node, const int &iter)
{
	std::uniform_real_distribution<double> rand_x, rand_y;
	rand_x = std::uniform_real_distribution<double>(XMIN, XMAX);
	rand_y = std::uniform_real_distribution<double>(YMIN, YMAX);
	std::default_random_engine e(iter*time(0));

	*rand_node = Vehicle::Node(rand_x(e), rand_y(e), 0, 0);
}

int RRT::RTG_RRT::grow(Vehicle::Node *new_node, Collision::collision *collimap)
{
	int result = 0;

	Index near_node_index;
	nearest_search(*new_node, &near_node_index);

	double delta = atan2(new_node->y - (*tree)[near_node_index]._node()->y, new_node->x - (*tree)[near_node_index]._node()->x);
	new_node->reset((*tree)[near_node_index]._node()->x + step*cos(delta), (*tree)[near_node_index]._node()->y + step*sin(delta), delta);

	if (new_node->x >= dest->_node()->x - 1 && new_node->x <= dest->_node()->x + 1 && new_node->y >= dest->_node()->y - 1 && new_node->y <= dest->_node()->y + 1)
	{
		new_node->reset(dest->_node()->x, dest->_node()->y);
		if (!collimap->iscollision((*tree)[near_node_index]._node(), *new_node))
		{
			dest->_index(tree->size());
			(*tree)[near_node_index]._successor(dest->_index());
			dest->_predecessor(near_node_index);
			kd_insert(dest, near_node_index);
			tree->push_back(dest);
			result = 2;
			root = &(*tree)[0];
		}
	}
	else
	{
		if (!collimap->iscollision((*tree)[near_node_index]._node(), *new_node))
		{
			Index new_index = tree->size();
			tree->emplace_back(*new_node, new_index, near_node_index, max_size + 1);
			(*tree)[near_node_index]._successor(new_index);
			kd_insert(&(*tree)[new_index], near_node_index);
			result = 1;
		}
	}

	return result;
}

void RRT::RTG_RRT::path2tree(const Index &predecessor, vector<Vehicle::Node> *path)
{
	int size = tree->size();
	tree->rbegin()->_successor(size);
	tree->emplace_back(*path->begin(), size++, predecessor, max_size);
	
	for (auto n = path->begin() + 1; n != path->end(); n++)
	{
		tree->rbegin()->_successor(size);
		tree->emplace_back(*n, size, size - 1, max_size);		
		size++;
	}
	
	if (*(path->rbegin()) == *dest->_node())
	{
		dest->_index(size - 1);
		dest->_predecessor(size - 2);
	}
}

int RRT::RTG_RRT::search(Collision::collision *collimap, vector<double> *L_theta, const double l, const double w, const double r)
{
	double le = 0;
	if (l == 0 && w == 0 && r == 0)
	{
		if (force_extend(L_theta, collimap, &le) == 2)
			return 2;
	}
	else
	{
		if (force_extend(l, w, r, L_theta, collimap, &le) == 2)
			return 2;
	}

	int iter = 1;
	Vehicle::Node rand_node;
	double lambda = 0.2f;
	int is_extend;
	int extended_num(0), unextended_num(0); 
	for (; iter < max_size; iter++)
	{
		std::uniform_real_distribution<double> rand = std::uniform_real_distribution<double>(0.f, 1.f);
		std::default_random_engine e(iter*time(0));
		double rand_lambda = rand(e);
		if (rand_lambda < lambda)
		{
			vector<Vehicle::Node> *path = new vector<Vehicle::Node>;
			is_extend = force_extend(L_theta, collimap, &le);
			switch (is_extend)
			{
			case 0:
				extended_num = 0;
				unextended_num++;
				_lambda_ss(&lambda, unextended_num);
				break;
			case 1:
				unextended_num = 0;
				extended_num++;
				_lambda_ss(&lambda, extended_num, le);
			case 2:
				break;
			}				
		}
		else
		{
			rand_select(&rand_node, iter);
			is_extend = grow(&rand_node, collimap);
		}
		if (is_extend == 2)
			break;
	}
	if (is_extend == 2)
		return 1;
	else
		return 0;
}

//flag=1:lanechangeleft;flag=2:lanechangeright;flag=3:lanechangeleftc;flag=4:lanechangerightc
//flag=5:leftturn;flag=6:rightturn;flag=7:leftturnc;flag=8:Uturn
//2:success 1:partially extension 0:failure
int RRT::RTG_RRT::force_extend(vector<double> *L_theta, Collision::collision *collimap, double *le)
{
	Index near_node_index;
	nearest_search(dest->_node(), &near_node_index);

	Vehicle::Node startnode = (*tree)[near_node_index]._node();
	
	vector<double> *bound_condition = new vector<double>{ dest->_node()->x - startnode.x, dest->_node()->y - startnode.y, dest->_node()->theta - startnode.theta };
	vector<double> *control = new vector<double>;
	bool is_force_exist;
	if ((*bound_condition)[2] >= THETA_min_L && (*bound_condition)[2] <= THETA_max_L)
	{
		if ((*bound_condition)[1] > 0)
			is_force_exist = Trajectory::lanechange(1, *L_theta, *bound_condition, control);
		else
			is_force_exist = Trajectory::lanechange(-1, *L_theta, *bound_condition, control);
	}
	else if ((*bound_condition)[2] >= THETA_min_U && (*bound_condition)[2] <= THETA_max_U)
	{
		is_force_exist = Trajectory::U_turn(*L_theta, *bound_condition, control);
	}
	else
	{
		if ((*bound_condition)[1] > 0)
			is_force_exist = Trajectory::turn(1, *L_theta, *bound_condition, control);
		else
			is_force_exist = Trajectory::turn(-1, *L_theta, *bound_condition, control);
	}
	
	if (is_force_exist == false)
		return 0;
	else
	{
		vector<Vehicle::Node> *path = new vector<Vehicle::Node>;
		int is_force_extend = Trajectory::is_force_extend_safe(startnode, step, *control, collimap, le, path);
		if (is_force_extend != 0)
			path2tree(near_node_index, path);
		delete path;
		return is_force_extend;
	}
}

int RRT::RTG_RRT::force_extend(const double &l, const double &w, const double &r, vector<double> *L_theta, Collision::collision *collimap, double *le)
{
	Index near_node_index;
	nearest_search(dest->_node(), &near_node_index);

	Vehicle::Node startnode = (*tree)[near_node_index]._node();
	Vehicle::Node endnode = Vehicle::Node(dest->_node()->x - startnode.x, dest->_node()->y - startnode.y, dest->_node()->theta - startnode.theta);

	vector<double> *bound_condition = new vector<double>{ dest->_node()->x - startnode.x, dest->_node()->y - startnode.y, dest->_node()->theta - startnode.theta };
	vector<double> *control = new vector<double>;
	bool is_force_exist;
	vector<double> *constraints = new vector<double>{ l, w };
	if (r != 0)
	{
		constraints->emplace_back(r);
		is_force_exist = Trajectory::turn_c(*L_theta, *bound_condition, *constraints, control);
		if (is_force_exist == false)
		{
			if ((*bound_condition)[1] > 0)
				is_force_exist = Trajectory::turn(1, *L_theta, *bound_condition, control);
			else
				is_force_exist = Trajectory::turn(-1, *L_theta, *bound_condition, control);
		}
	}
	else
	{
		if ((*bound_condition)[2] >= THETA_min_L && (*bound_condition)[2] <= THETA_max_L)
		{
			if ((*bound_condition)[1] > 0)
			{
				is_force_exist = Trajectory::lanechange_c(1, *L_theta, *bound_condition, *constraints, control);
				if (is_force_exist == false)
					is_force_exist = Trajectory::lanechange(1, *L_theta, *bound_condition, control);
			}
			else
			{
				is_force_exist = Trajectory::lanechange_c(-1, *L_theta, *bound_condition, *constraints, control);
				if (is_force_exist == false)
					is_force_exist = Trajectory::lanechange(-1, *L_theta, *bound_condition, control);
			}
		}
		else
		{
			is_force_exist = Trajectory::U_turn_c(*L_theta, *bound_condition, *constraints, control);
			if (is_force_exist == false)
				is_force_exist = Trajectory::U_turn(*L_theta, *bound_condition, control);
		}
	} //在有障碍物计算出现错误时，将障碍物去除然后再计算从而最大限度的获得所需路径
	
	if (is_force_exist == 0)
		return 0;
	else
	{
		vector<Vehicle::Node> *path = new vector<Vehicle::Node>;
		int is_force_extend = Trajectory::is_force_extend_safe(startnode, step, *control, collimap, le, path);
		if (is_force_extend != 0)
			path2tree(near_node_index, path);
		delete path;
		return is_force_extend;
	}
}

void RRT::RTG_RRT::_lambda_ss(double *lambda, const int &num, const double le)
{
	static double Le = 0.f;
	double gama = 0.5f;
	if (le == 0.f)
		*lambda *= expf(-0.5f / num);
	else
	{
		Le = std::max(Le, le);
		*lambda = *lambda + (1 - *lambda)*(gama*expf(-0.5f / num) + (1.f - gama)*le / Le);
	}
}

void RRT::RTG_RRT::getpath()
{
	double sg = 0.;
	Vector6d coeff;
	for (auto i = dest->_index(); i != 0;)
	{		
		route_tree->push_back(*((*tree)[i]._node()));
		i = (*tree)[i]._predecessor();
	}
	route_tree->push_back(*(root->_node()));
	std::reverse(route_tree->begin(), route_tree->end());
}
