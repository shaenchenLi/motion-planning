#include "RRT.h"

void RRT::RTG_RRT::kd_tree(Index* kd_father, const Vehicle::Node &new_node)
{
	*kd_father = 0;
	Index old = 0;
 	while (*kd_father != max_size+1)
	{
		old = *kd_father;
		switch ((*tree)[*kd_father]._deep())
		{
		case 0:
			if (norm(new_node.x, XMAX, XMIN) > norm((*tree)[*kd_father]._node()->x, XMAX, XMIN))
				*kd_father = (*tree)[*kd_father]._right();
			else
				*kd_father = (*tree)[*kd_father]._left();
			break;
		case 1:
			if (norm(new_node.y, dest->_node()->y, 0) > norm((*tree)[*kd_father]._node()->y, dest->_node()->y, 0))
				*kd_father = (*tree)[*kd_father]._right();
			else
				*kd_father = (*tree)[*kd_father]._left();
			break;
		}
	}
	*kd_father = old;
}

void RRT::RTG_RRT::kd_tree(RRT_Node *new_node, const Index &insert_index) 
{
	new_node->_kd_father(insert_index);

	switch ((*tree)[insert_index]._deep())
	{
	case 0:
		if (norm(new_node->_node()->x, XMAX, XMIN) > norm((*tree)[insert_index]._node()->x, XMAX, XMIN))
			(*tree)[insert_index]._right(new_node->_index());
		else
			(*tree)[insert_index]._left(new_node->_index());
		break;
	case 1:
		if (norm(new_node->_node()->y, dest->_node()->y, 0) > norm((*tree)[insert_index]._node()->y, dest->_node()->y, 0))
			(*tree)[insert_index]._right(new_node->_index());
		else
			(*tree)[insert_index]._left(new_node->_index());
		break;
	}

	new_node->_deep((*tree)[insert_index]._deep() + 1);
}

void RRT::RTG_RRT::nearest_search(Index* near_node, const Vehicle::Node &node)
{
	if (!tree->empty())
		kd_tree(near_node, node);
	else
		*near_node = HUGE_VAL;
}

void RRT::RTG_RRT::rand_select(Vehicle::Node *rand_node, const int &iter)
{
	std::uniform_real_distribution<float> rand_x, rand_y;
	rand_x = std::uniform_real_distribution<float>(XMIN, XMAX);
	rand_y = std::uniform_real_distribution<float>(YMIN, YMAX);
	std::default_random_engine e(iter*time(0));

	*rand_node = Vehicle::Node(rand_x(e), rand_y(e), 0, 0);
}

int RRT::RTG_RRT::grow(Vehicle::Node *new_node, Collision::collision *collimap)
{
	int result = 0;

	Index near_node_index;
	nearest_search(&near_node_index, *new_node);

	float delta = atan2f(new_node->y - (*tree)[near_node_index]._node()->y, new_node->x - (*tree)[near_node_index]._node()->x);
	new_node->reset((*tree)[near_node_index]._node()->x + step*cos(delta), (*tree)[near_node_index]._node()->y + step*sin(delta), delta);

	if (new_node->x >= dest->_node()->x - 1 && new_node->x <= dest->_node()->x + 1 && new_node->y >= dest->_node()->y - 1 && new_node->y <= dest->_node()->y + 1)
	{
		new_node->reset(dest->_node()->x, dest->_node()->y);
		if (!collimap->iscollision((*tree)[near_node_index]._node(), *new_node))
		{
			dest->_index(tree->size());
			(*tree)[near_node_index]._successor(dest->_index());
			dest->_predecessor(near_node_index);
			kd_tree(dest, near_node_index);
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
			kd_tree(&(*tree)[new_index], near_node_index);
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

int RRT::RTG_RRT::search(Collision::collision *collimap, vector<float> *L_theta, const float l, const float w, const float r)
{
	float le = 0;
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
	float lambda = 0.2f;
	int is_extend;
	int extended_num(0), unextended_num(0); 
	for (; iter < max_size; iter++)
	{
		std::uniform_real_distribution<float> rand = std::uniform_real_distribution<float>(0.f, 1.f);
		std::default_random_engine e(iter*time(0));
		float rand_lambda = rand(e);
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
int RRT::RTG_RRT::force_extend(vector<float> *L_theta, Collision::collision *collimap, float *le)
{
	Index near_node_index;
	nearest_search(&near_node_index, dest->_node());

	Vehicle::Node startnode = (*tree)[near_node_index]._node();
	
	vector<float> *bound_condition = new vector<float>{ dest->_node()->x - startnode.x, dest->_node()->y - startnode.y, dest->_node()->theta - startnode.theta };
	vector<float> *control = new vector<float>;
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

int RRT::RTG_RRT::force_extend(const float &l, const float &w, const float &r, vector<float> *L_theta, Collision::collision *collimap, float *le)
{
	Index near_node_index;
	nearest_search(&near_node_index, dest->_node());

	Vehicle::Node startnode = (*tree)[near_node_index]._node();
	Vehicle::Node endnode = Vehicle::Node(dest->_node()->x - startnode.x, dest->_node()->y - startnode.y, dest->_node()->theta - startnode.theta);

	vector<float> *bound_condition = new vector<float>{ dest->_node()->x - startnode.x, dest->_node()->y - startnode.y, dest->_node()->theta - startnode.theta };
	vector<float> *control = new vector<float>;
	bool is_force_exist;
	vector<float> *constraints = new vector<float>{ l, w };
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

void RRT::RTG_RRT::_lambda_ss(float *lambda, const int &num, const float le)
{
	static float Le = 0.f;
	float gama = 0.5f;
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
	float sg = 0.;
	Vector6f coeff;
	for (auto i = dest->_index(); i != 0;)
	{		
		route_tree->push_back(*((*tree)[i]._node()));
		i = (*tree)[i]._predecessor();
	}
	route_tree->push_back(*(root->_node()));
	std::reverse(route_tree->begin(), route_tree->end());
}
