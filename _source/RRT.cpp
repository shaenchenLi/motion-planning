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
				if ((*tree)[curr]._right() != -1)
					curr = (*tree)[curr]._right();
				else
				{
					(*tree)[curr]._right(new_node->_index());
					flag = 1;
				}
			}
			else
			{
				if ((*tree)[curr]._left() != -1)
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
				if ((*tree)[curr]._right() != -1)
					curr = (*tree)[curr]._right();
				else
				{
					(*tree)[curr]._right(new_node->_index());
					flag = 1;
				}
			}
			else
			{
				if ((*tree)[curr]._left() != -1)
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
	if (tree->size() <= kNN_NUM)
	{
		for (auto &i : *tree)
			kNN->insert({ Trajectory::dist(node, i._node()), i._index() });
		return;
	}
	DIS_TO_NODE *BPQ = new DIS_TO_NODE;
	std::set<Index> *BPQ_old = new std::set<Index>;

	Index curr = 0;
	int bt_num = 0;
	while (1)
	{
		Index old = 0;
		while (curr != -1)
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
							if (curr == -1)
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
							if (curr == -1)
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
							if (curr == -1)
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
							if (curr == -1)
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
		else
			break;
	}
	delete BPQ, BPQ_old;
}

void RRT::RTG_RRT::nearest_search(const Vehicle::Node &node, Index* near_node)
{
	DIS_TO_NODE *kNN = new DIS_TO_NODE;
	kNN_search(node, kNN);

	double near_theta;
	Index near_node_tmp;
	double min_dist = 1000;
	double delta_theta, dist;
	for (auto &i : *kNN)
	{
		near_theta = atan2(node.y - (*tree)[i.second]._node()->y, node.x - (*tree)[i.second]._node()->x);
		delta_theta = (*tree)[i.second]._node()->theta - near_theta;
		Trajectory::norm_theta_pi(&delta_theta);
		dist = i.first + PI - delta_theta;
		if (dist < min_dist)
		{
			min_dist = dist;
			near_node_tmp = i.second;
		}
	}
	*near_node = near_node_tmp;
	delete kNN;
}

void RRT::RTG_RRT::near_search(const Vehicle::Node &node, vector<Index>* near_node)
{
	DIS_TO_NODE *kNN = new DIS_TO_NODE;
	kNN_search(node, kNN);

	DIS_TO_NODE *NN = new DIS_TO_NODE;
	double near_theta;
	double delta_theta;// dist;
	for (auto &i : *kNN)
	{
		near_theta = atan2(node.y - (*tree)[i.second]._node()->y, node.x - (*tree)[i.second]._node()->x);
		delta_theta = (*tree)[i.second]._node()->theta - near_theta;
		Trajectory::norm_theta_pi(&delta_theta);
		NN->insert({ i.first + PI - delta_theta, i.second });
	}
	near_node->clear();
	for (auto i = NN->rbegin(); i != NN->rend(); i++)
		near_node->emplace_back(i->second);
	delete kNN;
	delete NN;
}

void RRT::RTG_RRT::rand_normal(const int &iter, Vehicle::Node *rand_node)
{
	std::uniform_real_distribution<double> rand_x, rand_y;
	rand_x = std::uniform_real_distribution<double>(XMIN, XMAX);
	rand_y = std::uniform_real_distribution<double>(YMIN, YMAX);
	std::default_random_engine e_x(iter*time(0)), e_y(iter*time(0) * 2);

	*rand_node = Vehicle::Node(rand_x(e_x), rand_y(e_y), 0, 0);
}

void RRT::RTG_RRT::rand_gaussian(const int &iter, vector<double> *gaus_para, Point2D *refer_point, Vehicle::Node *rand_node)
{
	std::uniform_real_distribution<double> n;
	std::default_random_engine r_e(iter*time(0)), theta_e(2 * iter*time(0));
	double r = n(r_e)*(*gaus_para)[0] + (*gaus_para)[1];
	double theta = n(theta_e)*(*gaus_para)[2] + (*gaus_para)[3];

	*rand_node = Vehicle::Node(refer_point->x + r*cos(theta), refer_point->y + r*sin(theta), 0);
}	

//gaus_para={sigma_r,r0,sigma_theta,theta0}
void RRT::RTG_RRT::gaussian_para(const int &type, vector<double> *gaus_para)
{
	double sigma_r, sigma_theta, r0, theta0;
	int sign;
	double delta = 3.;
	switch (type)
	{
	case 0: //无驾驶行为导向
		r0 = step;
		sigma_r = Trajectory::dist(root->_node(), dest->_node()) + delta;
		sigma_theta = PI / 2;// 2 * PI / 3;
		theta0 = atan2(dest->_node()->y - root->_node()->y, dest->_node()->x - root->_node()->x) - 0.5*sigma_theta;
		cout << sigma_r << " " << r0 << " " << sigma_theta << " " << theta0 << endl;
		break;
	case 1: //换道
		sign = root->_node()->y < dest->_node()->y ? 1 : -1;
		sigma_r = Trajectory::dist(root->_node(), dest->_node()) + delta;
		r0 = 0.5*step;
		theta0 = atan2(dest->_node()->y - sign*0.75*LANE_WIDTH - root->_node()->y, dest->_node()->x - root->_node()->x);
		sigma_theta = atan2(dest->_node()->y + 0.75*sign*LANE_WIDTH - root->_node()->y, dest->_node()->x - root->_node()->x) - theta0;
		Trajectory::norm_theta_0_2pi(&sigma_theta);
		break;
	case 4: //直线
		sigma_r = Trajectory::dist(root->_node(), dest->_node()) + delta;
		r0 = step;
		sigma_theta = 0.005*PI;
		theta0 = atan2(dest->_node()->y - root->_node()->y, dest->_node()->x - root->_node()->x) - 0.5*sigma_theta;
		break;
	}
	*gaus_para = { sigma_r, r0, sigma_theta, theta0 };
}

void RRT::RTG_RRT::gaussian_para_turn(const double &l, const int &i, vector<double> *gaus_para, Point2D *refer_point)
{
	double sigma_r, sigma_theta, r0(0), theta0;
	double delta = 3.;
	Index near;
	int sign = root->_node()->y < dest->_node()->y ? 1 : -1;
	switch (i)
	{
	case 0:
		r0 = step;
		sigma_r = dest->_node()->x - root->_node()->x + delta;
		sigma_theta = PI / 18;
		theta0 = root->_node()->theta - 0.5*sigma_theta;
		break;
	case 1:		
		nearest_search(Vehicle::Node(dest->_node()->x, root->_node()->y, 0, 0), &near);
		refer_point->reset((*tree)[near]._node()->x, (*tree)[near]._node()->y);
		r0 = step;
		sigma_r = Trajectory::dist(refer_point->x, refer_point->y, dest->_node()->x, dest->_node()->y) + delta + step;
		theta0 = atan2(root->_node()->y - sign*0.5*LANE_WIDTH - refer_point->y, root->_node()->x - refer_point->x);
		sigma_theta = atan2(dest->_node()->y - refer_point->y, dest->_node()->x + 0.75*LANE_WIDTH - refer_point->x) - theta0;
		Trajectory::norm_theta_0_2pi(&sigma_theta);
		break;
	case 2:
		refer_point->reset(root->_node()->x, root->_node()->y);
		sigma_r = Trajectory::dist(root->_node(), dest->_node()) + 2 * delta;
		r0 = 0.5*(dest->_node()->x - root->_node()->x);
		theta0 = atan2(-sign*2., dest->_node()->x - root->_node()->x);
		sigma_theta = atan2(dest->_node()->y + sign*step - root->_node()->y, dest->_node()->x - root->_node()->x) - theta0;
		Trajectory::norm_theta_0_2pi(&sigma_theta);
		break;
	}
	*gaus_para = { sigma_r, r0, sigma_theta, theta0 };
}

void RRT::RTG_RRT::gaussian_para_Uturn(const double &l, const int &i, vector<double> *gaus_para, Point2D *refer_point)
{
	double sigma_r, sigma_theta, r0(0), theta0;
	double delta = 3.;
	Index near;
	switch (i)
	{
	case 0:
		sigma_r = l == 0 ? delta : l;
		sigma_theta = PI / 18;
		theta0 = root->_node()->theta - 0.5*sigma_theta;
		break;
	case 1:
		nearest_search(Vehicle::Node(root->_node()->x + (l == 0 ? delta : l), 0.5*(root->_node()->y + dest->_node()->y), 0, 0), &near);
		refer_point->reset((*tree)[near]._node()->x, (*tree)[near]._node()->y);
		sigma_r = 1.25*std::abs(root->_node()->y - dest->_node()->y);
		theta0 = 0;
		sigma_theta = PI;
		break;
	case 2:
		nearest_search(Vehicle::Node(root->_node()->x + (l == 0 ? delta : l), dest->_node()->y, 0, 0), &near);
		refer_point->reset((*tree)[near]._node()->x, (*tree)[near]._node()->y);
		sigma_r = Trajectory::dist((*tree)[near]._node()->x, (*tree)[near]._node()->y, dest->_node()->x, dest->_node()->y) + delta;
		theta0 = atan2(dest->_node()->y - 0.75*LANE_WIDTH - (*tree)[near]._node()->y, dest->_node()->x - (*tree)[near]._node()->x);
		sigma_theta = atan2(dest->_node()->y + 0.75*LANE_WIDTH - (*tree)[near]._node()->y, dest->_node()->x - (*tree)[near]._node()->x) - theta0;
		Trajectory::norm_theta_0_2pi(&sigma_theta);
		break;
	case 3:
		refer_point->reset(root->_node()->x, root->_node()->y);
		sigma_r = (l == 0 ? delta : l) + 1 / std::min(KMAX, std::abs(KMIN)) + delta;
		theta0 = 0;
		if ((dest->_node()->x - root->_node()->x) != 0)
			sigma_theta = atan2(dest->_node()->x - root->_node()->x, dest->_node()->y - root->_node()->y + 0.75*LANE_WIDTH);
		else
			sigma_theta = PI;
		break;
	}
	*gaus_para = { sigma_r, r0, sigma_theta, theta0 };
}

int RRT::RTG_RRT::grow(Vehicle::Node *new_node)
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
			tree->emplace_back(*new_node, new_index, near_node_index);
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
	tree->emplace_back(*path->begin(), size++, predecessor);
	
	for (auto n = path->begin() + 1; n != path->end(); n++)
	{
		tree->rbegin()->_successor(size);
		tree->emplace_back(*n, size, size - 1);		
		size++;
	}
	
	if (*(path->rbegin()) == *dest->_node())
	{
		dest->_index(size - 1);
		dest->_predecessor(size - 2);
	}
}

//type:0无驾驶行为导向 1换道 2转弯 3掉头 4直行
int RRT::RTG_RRT::search(const int &type, const double l, const double w/*, const double r*/)
{
	double le = 0;
	if (l == 0 && w == 0/* && r == 0*/)
	{
		if (force_extend(type, &le) == 2)
			return 2;
	}
	else
	{
		if (force_extend(type, l, w, &le) == 2)
			return 2;
	}
	vector<double> *gaus_para = new vector<double>(4);
	Point2D *refer_point = new Point2D(root->_node()->x, root->_node()->y);
	if (flag == 1)
	{
		switch (type)
		{
		case 3:
			gaussian_para_Uturn(l, 0, gaus_para, refer_point);
			break;
		case 2:
			gaussian_para_turn(l, 0, gaus_para, refer_point);
			break;
		default:
			gaussian_para(type, gaus_para);
			break;
		}
	}

	int iter = 1, rand_num = 0;
	Vehicle::Node rand_node;
	double lambda = 0.2;
	int is_extend;
	int extended_num(0), unextended_num(0); 
	for (; iter < max_iter; iter++)
	{
		std::uniform_real_distribution<double> rand;
		std::default_random_engine e(iter*time(0));
		double rand_lambda = rand(e);
		if (rand_lambda < lambda)
		{
			vector<Vehicle::Node> *path = new vector<Vehicle::Node>;
			is_extend = force_extend(1,&le);
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
			rand_num++;
			
			if (flag == 0)
				rand_normal(iter, &rand_node);
			else
			{
				switch (type)
				{
				case 3:
					switch (rand_num)
					{
					case U_TURN_RAND_STEP1:
						gaussian_para_Uturn(l, 1, gaus_para, refer_point);
						break;
					case U_TURN_RAND_STEP2:
						gaussian_para_Uturn(l, 2, gaus_para, refer_point);
						break;
					case U_TURN_RAND_STEP3:
						gaussian_para_Uturn(l, 3, gaus_para, refer_point);
						break;
					default:
						break;
					}
					break;
				case 2:
					switch (rand_num)
					{
					case TURN_RAND_STEP1:
						gaussian_para_turn(l, 1, gaus_para, refer_point);
						break;
					case TURN_RAND_STEP2:
						gaussian_para_turn(l, 2, gaus_para, refer_point);
						break;
					default:
						break;
					}
					break;
				default:
					break;
				}
				
				rand_gaussian(iter, gaus_para, refer_point, &rand_node);
			}

			is_extend = grow(&rand_node);
		}
			
		if (is_extend == 2)
			break;
	}
	if (is_extend == 2)
		return 1;
	else
		return 0;
}

int RRT::RTG_RRT::search_count(const int &type, int *count, const double l, const double w)
{
	double le = 0;
	if (l == 0 && w == 0)
	{
		if (force_extend(type, &le) == 2)
			return 2;
	}
	else
	{
		if (force_extend(type, l, w, &le) == 2)
			return 2;
	}
	vector<double> *gaus_para = new vector<double>(4);
	Point2D *refer_point = new Point2D(root->_node()->x, root->_node()->y);
	if (flag == 1)
	{
		switch (type)
		{
		case 3:
			gaussian_para_Uturn(l, 0, gaus_para, refer_point);
			break;
		case 2:
			gaussian_para_turn(l, 0, gaus_para, refer_point);
			break;
		default:
			gaussian_para(type, gaus_para);
			break;
		}
	}

	int iter = 1, rand_num = 0;
	Vehicle::Node rand_node;
	double lambda = 0.2;
	int is_extend;
	int extended_num(0), unextended_num(0);
	for (; iter < max_iter; iter++)
	{
		std::uniform_real_distribution<double> rand;
		std::default_random_engine e(iter*time(0));
		double rand_lambda = rand(e);
		if (rand_lambda < lambda)
		{
			vector<Vehicle::Node> *path = new vector<Vehicle::Node>;
			is_extend = force_extend(1, &le);
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
			rand_num++;

			if (flag == 0)
				rand_normal(iter, &rand_node);
			else
			{
				switch (type)
				{
				case 3:
					switch (rand_num)
					{
					case U_TURN_RAND_STEP1:
						gaussian_para_Uturn(l, 1, gaus_para, refer_point);
						break;
					case U_TURN_RAND_STEP2:
						gaussian_para_Uturn(l, 2, gaus_para, refer_point);
						break;
					case U_TURN_RAND_STEP3:
						gaussian_para_Uturn(l, 3, gaus_para, refer_point);
						break;
					default:
						break;
					}
					break;
				case 2:
					switch (rand_num)
					{
					case TURN_RAND_STEP1:
						gaussian_para_turn(l, 1, gaus_para, refer_point);
						break;
					case TURN_RAND_STEP2:
						gaussian_para_turn(l, 2, gaus_para, refer_point);
						break;
					default:
						break;
					}
					break;
				default:
					break;
				}

				rand_gaussian(iter, gaus_para, refer_point, &rand_node);
			}

			is_extend = grow(&rand_node);
		}

		if (is_extend == 2)
			break;
	}
	*count = iter;
	if (is_extend == 2)
		return 1;
	else
		return 0;
}

//flag=1:lanechange;flag=2:turn;flag=3:Uturn flag=4:straight
//2:success 1:partially extension 0:failure
int RRT::RTG_RRT::force_extend(const int &type, double *le)
{
	Index near_node_index;
	nearest_search(*dest->_node(), &near_node_index);

	Vehicle::Node startnode = (*tree)[near_node_index]._node();
	
	vector<double> *bound_condition = new vector<double>{ dest->_node()->x - startnode.x, dest->_node()->y - startnode.y, dest->_node()->theta - startnode.theta };
	vector<double> *control = new vector<double>;
	bool is_force_exist;
	switch (type)
	{
	case 0:  case 1:
		is_force_exist = Trajectory::lanechange(startnode, *dest->_node(), control);
		break;
	case 2:
		is_force_exist = Trajectory::turn(startnode, *dest->_node(), control);
		break;
	case 3:
		is_force_exist = Trajectory::U_turn(startnode, *dest->_node(), control);
		break;
	case 4:
		is_force_exist = Trajectory::straight(*bound_condition, control);
		break;
	}
	
	if (is_force_exist == false)
		return 0;
	else
	{
		vector<Vehicle::Node> *path = new vector<Vehicle::Node>;
		int is_force_extend = Trajectory::is_force_extend_safe(startnode, step, *control, le, path);
		if (is_force_extend != 0)
			path2tree(near_node_index, path);
		delete path;
		return is_force_extend;
	}
}

int RRT::RTG_RRT::force_extend(const int &type, const double &l, const double &w, double *le)
{
	Index near_node_index;
	nearest_search(*dest->_node(), &near_node_index);

	Vehicle::Node startnode = (*tree)[near_node_index]._node();
	Vehicle::Node endnode = Vehicle::Node(dest->_node()->x - startnode.x, dest->_node()->y - startnode.y, dest->_node()->theta - startnode.theta);

	vector<double> *bound_condition = new vector<double>{ dest->_node()->x - startnode.x, dest->_node()->y - startnode.y, dest->_node()->theta - startnode.theta };
	vector<double> *control = new vector<double>;
	bool is_force_exist;
	vector<double> *constraints = new vector<double>{ l, w };
	switch (type)
	{
	case 1:
		is_force_exist = Trajectory::lanechange_c(/*1,  *L_theta, */ startnode, *dest->_node(), *constraints, control);
		if (is_force_exist == false)
			is_force_exist = Trajectory::lanechange(/* 1, *L_theta, */ startnode, *dest->_node(), control);
		break;
	case 2:
		is_force_exist = Trajectory::turn_c(startnode, *dest->_node(), *constraints, control);
		if (is_force_exist == false)
			is_force_exist = Trajectory::lanechange(startnode, *dest->_node(), control);
		break;
	case 3:
		is_force_exist = Trajectory::U_turn_c(startnode, *dest->_node(), *constraints, control);
		if (is_force_exist == false)
			is_force_exist = Trajectory::U_turn(startnode, *dest->_node(), control);
		break;
	case 0:
		break;
	}//出现错误时，将障碍物去除然后再计算从而最大限度的获得所需路径
	
	if (is_force_exist == 0)
		return 0;
	else
	{
		vector<Vehicle::Node> *path = new vector<Vehicle::Node>;
		int is_force_extend = Trajectory::is_force_extend_safe(startnode, step, *control, /*collimap, */le, path);
		if (is_force_extend != 0)
			path2tree(near_node_index, path);
		delete path;
		return is_force_extend;
	}
}

void RRT::RTG_RRT::_lambda_ss(double *lambda, const int &num, const double le)
{
	static double Le = 0.;
	double gama = 0.5;
	if (le == 0.f)
		*lambda *= expf(-0.5f / num);
	else
	{
		Le = std::max(Le, le);
		*lambda = *lambda + (1 - *lambda)*(gama*expf(-0.5f / num) + (1.f - gama)*le / Le);
	}
}

void RRT::RTG_RRT::getpath(const Index &end)
{
	double sg = 0.;
	Vector6d coeff;
	for (auto i = end; i != 0;)
	{		
		route_tree->push_back(*((*tree)[i]._node()));
		i = (*tree)[i]._predecessor();
	}
	route_tree->push_back(*(root->_node()));
	std::reverse(route_tree->begin(), route_tree->end());
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

void RRT::RTG_RRT::_motion(const vector<Trajectory::State> &t)
{
	if (motion->empty())
	{
		motion->resize(t.size());
		memcpy(&(*motion)[0], &t[0], t.size()*sizeof(Trajectory::State));
	}
	else
	{
		int n = motion->size();
		motion->resize(t.size() + n);
		memcpy(&(*motion)[n], &t[0], t.size()*sizeof(Trajectory::State));
	}
}

void RRT::RTG_RRT::reset(const Index &new_root)
{
	*root = RRT_Node(*(*tree)[new_root]._node(), 0, 0);
	vector<RRT_Node>({ *root }).swap(*tree);
	vector<Vehicle::Node>().swap(*route_tree);
	
	if (dest->_node()->x - root->_node()->x > RRTSTEP_l || dest->_node()->y - root->_node()->y > RRTSTEP_l)
		step = RRTSTEP_l;
	else
		step = RRTSTEP_s;

	max_iter = MAXITER_2;
}

void RRT::RTG_RRT::_copy(const RRT::RTG_RRT &copied_tree)
{
	max_iter = copied_tree.max_iter;
	step = copied_tree.step;
	flag = copied_tree.flag;
	root->reset(*copied_tree.root);
	dest->reset(*copied_tree.dest);
	vector<RRT::RRT_Node>().swap(*tree);
	vector<Vehicle::Node>().swap(*route_tree);
	vector<Trajectory::State>().swap(*motion);
	for (auto i = copied_tree.tree->begin(); i != copied_tree.tree->end(); i++)
		tree->emplace_back(*i);

	for (auto i = copied_tree.route_tree->begin(); i != copied_tree.route_tree->end(); i++)
		route_tree->emplace_back(*i);

	for (auto i = copied_tree.motion->begin(); i != copied_tree.motion->end(); i++)
		motion->emplace_back(*i);
}