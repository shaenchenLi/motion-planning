#include "Trajectory.h"

double eps = 1e-6;

double Trajectory::norm_theta_0_2pi(const double &theta)
{
	double new_theta = std::fmod(theta, TWO_PI);
	if (new_theta < -PI)
		new_theta += TWO_PI;
	else if (new_theta>PI)
		new_theta -= TWO_PI;
	return new_theta;
}

void Trajectory::norm_theta_0_2pi(double *theta)
{
	*theta = std::fmod(*theta, TWO_PI);
	if (*theta < -PI)
		*theta += TWO_PI;
	else if (*theta>PI)
		*theta -= TWO_PI;
}//约束角度在 - pi到pi之间

void Trajectory::norm_theta_2pi(double *theta)
{
	norm_theta_0_2pi(theta);
	if (*theta < 0)
		*theta += 2 * PI;
}//约束角度在0到2*pi之间

void Trajectory::norm_theta_pi(double *theta)
{
	norm_theta_0_2pi(theta);
	if (*theta > 0)
		*theta = PI - *theta;
	else
		*theta = PI + *theta;
}//求夹角

double Trajectory::dist(const Vehicle::Node &n1, const Vehicle::Node &n2)
{
	return sqrt(pow(n1.x - n2.x, 2) + pow(n1.y - n2.y, 2));
}

double Trajectory::dist(const double &x1, const double &y1, const double &x2, const double &y2)
{
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

double Trajectory::dist(const position<double> &p)
{
	return sqrt(pow(p.x, 2) + pow(p.y, 2));
}

double Trajectory::dist(const position<double> &p1, const position<double> &p2)
{
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

void Trajectory::traj::_ctrl_points(const std::vector<Vehicle::Node> &route_tree)
{
	ctrl_points->clear();
	//vector<position<double>>().swap(*ctrl_points);
	ctrl_points->emplace_back(route_tree.begin()->x, route_tree.begin()->y);
	vector<double> L_min;// = new vector<double>;
	/*memcpy(&L_min, &L_theta, sizeof(L_theta));
	std::reverse(L_min.begin(), L_min.end());*/
	int N = (int)(L_theta.size() / 2);
	for (int i = N - 1; i >= 0; i--)
		L_min.push_back(L_theta[i]);
	L_min.emplace_back(0);      L_min.emplace_back(PI);

	double x, y, theta;

	std::vector<Vehicle::Node> tree_tmp = route_tree;
	std::vector<Vehicle::Node> route = { *tree_tmp.begin() };
	switch (tree_tmp.size())
	{
	case 1:
		x = tree_tmp.begin()->x + SAFESTEP * cos(tree_tmp.begin()->theta);
		y = tree_tmp.begin()->y + SAFESTEP * sin(tree_tmp.begin()->theta);
		ctrl_points->emplace_back(x, y);
		seg_end = State(x, y, tree_tmp.begin()->theta, 0);
		return;
	case 2:
		if (dist(*tree_tmp.begin(), *tree_tmp.rbegin()) <= RRTSTEP_s)
		{
			ctrl_points->emplace_back((2 * seg_begin._node()->x + seg_end._node()->x) / 3, (2 * seg_begin._node()->y + seg_end._node()->y) / 3);
			ctrl_points->emplace_back((seg_begin._node()->x + 2 * seg_end._node()->x) / 3, (seg_begin._node()->y + 2 * seg_end._node()->y) / 3);
			ctrl_points->emplace_back(seg_end._node()->x, seg_end._node()->y);
			return;
		}
		else
		{
			double x1, y1, x2, y2;
			x1 = tree_tmp.begin()->x + SAFESTEP * cos(tree_tmp.begin()->theta);
			y1 = tree_tmp.begin()->y + SAFESTEP * sin(tree_tmp.begin()->theta);
			x2 = tree_tmp.rbegin()->x - SAFESTEP * cos(tree_tmp.rbegin()->theta);
			y2 = tree_tmp.rbegin()->y - SAFESTEP * sin(tree_tmp.rbegin()->theta);
			if (dist(x1, y1, x2, y2) <= RRTSTEP_s)
			{
				ctrl_points->emplace_back((2 * seg_begin._node()->x + seg_end._node()->x) / 3, (2 * seg_begin._node()->y + seg_end._node()->y) / 3);
				ctrl_points->emplace_back((seg_begin._node()->x + 2 * seg_end._node()->x) / 3, (seg_begin._node()->y + 2 * seg_end._node()->y) / 3);
				ctrl_points->emplace_back(seg_end._node()->x, seg_end._node()->y);
				return;
			}
			else
			{
				route.emplace_back(x1, y1, tree_tmp.begin()->theta, 0.);
				ctrl_points->emplace_back(0.5*(x1 + route.begin()->x), 0.5*(y1 + route.begin()->y));
				ctrl_points->emplace_back(x, y);
				route.emplace_back(x2, y2, atan2(y2 - y1, x2 - x1), 0.);
				route.emplace_back(*tree_tmp.rbegin());
			}
		}
		break;
	default:
		for (auto node_a = tree_tmp.begin(); node_a != tree_tmp.end(); node_a++)
		{
			if (node_a == tree_tmp.begin())
			{
				x = tree_tmp.begin()->x + SAFESTEP * cos(tree_tmp.begin()->theta);
				y = tree_tmp.begin()->y + SAFESTEP * sin(tree_tmp.begin()->theta);
				route.emplace_back(x, y, tree_tmp.begin()->theta, 0.);
				(tree_tmp.begin() + 1)->reset(atan2(node_a->y - y, node_a->x - x));
				ctrl_points->emplace_back(0.5*(x + route.begin()->x), 0.5*(y + route.begin()->y));
				ctrl_points->emplace_back(x, y);
				continue;
			}
			else if (node_a == tree_tmp.end() - 1)
			{
				x = tree_tmp.rbegin()->x - SAFESTEP * cos(tree_tmp.rbegin()->theta);
				y = tree_tmp.rbegin()->y - SAFESTEP * sin(tree_tmp.rbegin()->theta);
				theta = atan2(y - route.rbegin()->y, x - route.rbegin()->x);
				double dis = dist(route.rbegin()->x, route.rbegin()->y, x, y);
				for (double s = SAFESTEP; s < dis; s += SAFESTEP)
				{
					if (iscollision(route.rbegin()->x + s*cos(theta), route.rbegin()->y + s*sin(theta), theta))
					{
						route.emplace_back(*(node_a - 1));
						break;
					}
				}
				route.emplace_back(x, y, atan2(y - route.rbegin()->y, x - route.rbegin()->x), 0.f);
				route.emplace_back(*tree_tmp.rbegin());
				break;
			}

			double dis = dist(*route.rbegin(), *node_a);
			double theta = atan2(node_a->y - route.rbegin()->y, node_a->x - route.rbegin()->x);
			for (double s = SAFESTEP; s < dis; s += SAFESTEP)
			{
				if (iscollision(route.rbegin()->x + s*cos(theta), route.rbegin()->y + s*sin(theta), theta))
				{
					route.emplace_back(*(node_a - 1));
					break;
				}
			}
		}
		break;
	}

	if (route.size() == 4)
	{
		if (dist(route[1], route[2]) < L_theta[1])
		{
			ctrl_points->pop_back();   ctrl_points->pop_back();
			ctrl_points->emplace_back((2 * seg_begin._node()->x + seg_end._node()->x) / 3, (2 * seg_begin._node()->y + seg_end._node()->y) / 3);
			ctrl_points->emplace_back((seg_begin._node()->x + 2 * seg_end._node()->x) / 3, (seg_begin._node()->y + 2 * seg_end._node()->y) / 3);
			ctrl_points->emplace_back(seg_end._node()->x, seg_end._node()->y);
			return;
		}
	}
	Vehicle::Node reference_node = *(route.begin());
	//double theta_min/*, L_min*/;
	int flag = 0; // 0:node++ 1:表示仍需要再次插入insert_node 2:end
	//auto a = route.begin();
	auto node_b = route.begin();
	node_b++;

	Vehicle::Node insert_node_tmp;
	Vehicle::Node node_behind = &*(route.rbegin());
	Vehicle::Node node_tmp = &*node_b;
	Vehicle::Node insert_node = &*node_b;
	std::map<double, Vehicle::Node> insert_node_set;
	auto insert_refer = node_tmp; //在新增结点之前，如果flag=1，需要比较与之前增加的结点（如果有）的优劣，则这个结点表示新增结点之前的之前结点，用于计算角度距离等
	double lf, lb, li, lfi;//lf为nodetmp和refer之间的距离，lb是nodetmp和node_b+1之间的距离，li为insert的相关距离，lfi表示insert_refer与之前结点的距离
	//std::vector<Vehicle::Node> insert_node_set{ *(route.end() - 2) };
	lfi = dist(node_tmp, *(node_b - 1));

	while (flag!=2)
	//for (auto node = route.begin() + 1; node != route.end() - 1; node++)
	{
		if (/*node_b == route.end() - 3 || */node_b == route.end() - 2) // the end of the ctrlpoints
		{
			if (flag == 0)
			{
				//reference_node = *node_b;
				//node_tmp = &*(++node_b);
				insert_refer = *(node_b + 1);
				lfi = dist(*node_b, *(node_b + 1));
				insert_node_set.clear();
				insert_node_set.insert({ 1000., *(route.end() - 2) });
				ctrl_points->pop_back();
				ctrl_points->pop_back();  //为了防止node处曲率超过需要插入新的node将原来的node弹出重新插入
			}

			double theta = node_tmp.theta - node_behind.theta;
			norm_theta_pi(&theta);
			lf = dist(reference_node, node_tmp);
			lb = dist(node_tmp, node_behind);
			double L_real = std::min(lf, lb);
			if (L_real < Vehicle::_L_min(theta))
			{
				flag = -1;
				double L_tmp, L_diff_tmp, L_diff = HUGE_VAL;
				//double dist_refer_to_node = dist(*node_tmp, *(node + 1));
				for (int i = 0; 2*i < N; i++)
				{
					if (L_min[2 * i] > lb)
					{
						if (2*i == N-2)
						{
							do
							{
								node_tmp.reset(node_tmp.x - 0.05*cos(node_behind.theta), node_tmp.y - 0.05*sin(node_behind.theta));
								node_tmp.reset(atan2(node_tmp.y - reference_node.y, node_tmp.x - reference_node.x));
								lb = dist(node_tmp, node_behind);
							}
							while (L_min[2 * i] > lb);
							/*flag = 1;
							insert_node = &Vehicle::Node(node_tmp->x + L_min[2 * i] * cos(node_tmp->theta), node_tmp->y + L_min[2 * i] * sin(node_tmp->theta), node_tmp->theta, 0.f);
							break;*/
						}
						else
							continue;
					}
					//theta = node_tmp->theta - (node_behind)->theta;
					//norm_theta_pi(&theta);
					double bbbbb = node_behind.theta; norm_theta_2pi(&bbbbb);
					double ttttt = node_tmp.theta; norm_theta_2pi(&ttttt);
					int is_clockwise = 1; //顺时针or逆时针 -1为逆时针
					if (ttttt < PI)
					{
						if (bbbbb<ttttt + PI&&bbbbb>ttttt)
							is_clockwise = -1;
					}
					else
					{
						if (bbbbb > ttttt&&bbbbb < ttttt - PI)
							is_clockwise = -1;
					}
					double insert_theta = node_behind.theta - is_clockwise*L_min[2 * i + 1];
					double insert_x = node_tmp.x + L_min[2 * i] * cos(insert_theta);
					double insert_y = node_tmp.y + L_min[2 * i] * sin(insert_theta);
					insert_node_tmp = &Vehicle::Node(insert_x, insert_y, atan2(insert_y - reference_node.y, insert_x - reference_node.x), 0.);
					node_tmp.reset(atan2(node_tmp.y - insert_y, node_tmp.x - insert_x));					
					double new_theta = insert_node_tmp.theta - node_tmp.theta;
					norm_theta_pi(&new_theta); // insert node's theta
					li = dist(insert_x, insert_y, reference_node.x, reference_node.y);
					if (Vehicle::is_node_effect(insert_node_tmp) && (!/*collimap->*/iscollision(node_tmp, insert_node_tmp)) && (!/*collimap->*/iscollision(insert_node_tmp, node_behind)))
					{
						L_tmp = Vehicle::_L_min(new_theta);
						if (std::min(L_min[2 * i], li) < L_tmp)
						{
							flag = 1;
							L_diff_tmp = L_tmp - std::min(L_min[2 * i], li);
							if (L_diff_tmp < L_diff)
							{
								insert_node = insert_node_tmp;
								L_diff = L_diff_tmp;
							}
						}
						else
						{
							flag = 2;
							insert_node = insert_node_tmp;
							break;
						}
					}
				}
				if (flag == -1)
				{
					if (Vehicle::is_node_effect(insert_node_tmp))
						cout << "insert_node_tmp isn't effective" << endl;
					if (/*collimap->*/iscollision(node_tmp, insert_node_tmp))
						cout << "the path from node_tmp to insert_node_tmp isn't safe" << endl;
					if (/*collimap->*/iscollision(insert_node_tmp, node_behind))
						cout << "the path from insert_node_tmp to node_behind isn't safe" << endl;

					if (insert_node_set.empty())
					{
						ctrl_points->emplace_back(0.5f*(reference_node.x + node_tmp.x), 0.5f*(reference_node.y + node_tmp.y));
						ctrl_points->emplace_back(node_tmp.x, node_tmp.y);
					}
					else
					{
						for (auto n = insert_node_set.begin(); n != insert_node_set.end(); n++)
						{
							ctrl_points->emplace_back(0.5*(reference_node.x + n->second.x), 0.5*(reference_node.y + n->second.y));
							ctrl_points->emplace_back(n->second.x, n->second.y);
							reference_node = n->second;
						}
					}
					break;
				}
				else if (flag == 2)
				{
					if (L_diff < insert_node_set.begin()->first)
						insert_node_set.insert({ L_diff, insert_node });
					//if ()
					//node_behind->reset(atan2(node_behind->y - insert_node->y, node_behind->x - insert_node->x));
					for (auto n = insert_node_set.begin(); n != insert_node_set.end(); n++)
					{
						ctrl_points->emplace_back(0.5*(reference_node.x + n->second.x), 0.5*(reference_node.y + n->second.y));
						ctrl_points->emplace_back(n->second.x, n->second.y);
						reference_node = n->second;
					}
					//ctrl_points->emplace_back(0.5*(before_end_insert_node.begin()->x + (route.end() - 2)->x), 0.5*(before_end_insert_node.begin()->y + (route.end() - 2)->y));
					//ctrl_points->emplace_back((route.end() - 2)->x, (route.end() - 2)->y);
					break;
				}
				else
				{
					li = Vehicle::_L_min(atan2(node_behind.y - insert_node.y, node_behind.x - insert_node.x) - insert_refer.theta) - std::min(lfi, dist(node_behind, insert_node));
					li += Vehicle::_L_min(insert_node.theta - atan2(node_behind.y - insert_node.y, node_behind.x - insert_node.x)) - std::min(dist(insert_node, reference_node), dist(node_behind, insert_node));
					if (li < std::min(L_diff, insert_node_set.rbegin()->first))
					{
						insert_node_set.erase(insert_node_set.rbegin()->first);
						insert_node_set.insert({ li, insert_node });
						node_tmp = insert_node;						
					}
					else if (L_diff < insert_node_set.begin()->first)
					{
						insert_node_set.insert({ L_diff, insert_node });
						insert_refer = node_behind;
						node_behind = node_tmp;
						lfi = dist(node_behind, insert_refer);
						node_tmp = insert_node;
						continue;
					}
					else
					{
						for (auto n = insert_node_set.begin(); n != insert_node_set.end(); n++)
						{
							ctrl_points->emplace_back(0.5*(reference_node.x + n->second.x), 0.5*(reference_node.y + n->second.y));
							ctrl_points->emplace_back(n->second.x, n->second.y);
							reference_node = n->second;
						}
						break;
					}

				}
			}
			else
			{
				/*if (insert_node_set.empty())
				{
					ctrl_points->emplace_back(0.5f*(reference_node.x + node_tmp.x), 0.5f*(reference_node.y + node_tmp.y));
					ctrl_points->emplace_back(node_tmp.x, node_tmp.y);
				}
				else
				{	*/				
					for (auto n = insert_node_set.begin(); n != insert_node_set.end(); n++)
					{
						ctrl_points->emplace_back(0.5*(reference_node.x + n->second.x), 0.5*(reference_node.y + n->second.y));
						ctrl_points->emplace_back(n->second.x, n->second.y);
						reference_node = n->second;
					}
				//	ctrl_points->emplace_back(0.5*(before_end_insert_node.begin()->x + (route.end() - 2)->x), 0.5*(before_end_insert_node.begin()->y + (route.end() - 2)->y));
				//	ctrl_points->emplace_back((route.end() - 2)->x, (route.end() - 2)->y);
				//}
				flag = 2;
				break;
			}
		}

		//if (flag == 0)
		//{
		//	node_tmp = &*(node + 1);
		//	//ctrl_points->emplace_back(0.5f*(reference_node.x + node_tmp.x), 0.5f*(reference_node.y + node_tmp.y));
		//	//ctrl_points->emplace_back(node_tmp.x, node_tmp.y);
		//	insert_node_set.clear();
		//}			
		//else
		//	node_tmp = insert_node;
				
		double theta = node_tmp.theta - (node_b + 1)->theta;
		//double theta = node_tmp->theta - (node + 1)->theta;
		norm_theta_pi(&theta);
		lf = dist(reference_node, node_tmp);
		lb = dist(node_tmp, *(node_b + 1));
		double L_real = std::min(lf, lb);
		if (L_real < Vehicle::_L_min(theta))
		{
			flag = -1;
			double L_tmp, L_diff_tmp, L_diff = HUGE_VALF;
			//double dist_refer_to_node = ;
			for (int i = 0; 2*i < N; i++)
			{
				if (L_min[2 * i] > dist(reference_node, node_tmp))
				{
					if (2*i == N-2)
					{
						do
						{
							node_tmp.reset(node_tmp.x + 0.05*cos(node_tmp.theta), node_tmp.y + 0.05*sin(node_tmp.theta));
							(node_b + 1)->reset(atan2((node_b + 1)->y - node_tmp.y, (node_b + 1)->x - node_tmp.x));
							lf = dist(reference_node, node_tmp);
						}
						while (L_min[2 * i] > lf);
						//insert_node = &Vehicle::Node(node_tmp->x + L_min[2 * i] * cos(node_tmp->theta), node_tmp->y + L_min[2 * i] * sin(node_tmp->theta), node_tmp->theta, 0.f);
						//flag = 1;
						
					}
					else
						continue;
				}	
				double bbbbb = (node_b + 1)->theta; norm_theta_2pi(&bbbbb);
				double ttttt = node_tmp.theta; norm_theta_2pi(&ttttt);
				int is_clockwise = 1; //顺时针or逆时针 -1为逆时针
				if (ttttt < PI)
				{
					if (bbbbb<ttttt + PI&&bbbbb>ttttt)
						is_clockwise = -1;
				}
				else
				{
					if (bbbbb > ttttt&&bbbbb < ttttt - PI)
						is_clockwise = -1;
				}
				double insert_theta = node_tmp.theta - is_clockwise*(PI - L_min[2 * i + 1]);
				double insert_x = node_tmp.x + L_min[2 * i] * cos(insert_theta);
				double insert_y = node_tmp.y + L_min[2 * i] * sin(insert_theta);
				insert_node_tmp = &Vehicle::Node(insert_x, insert_y, insert_theta, 0.f);
				double new_theta = insert_theta - atan2((node_b + 1)->y - insert_y, (node_b + 1)->x - insert_x); // insert node's theta
				norm_theta_pi(&new_theta);
				li = dist(insert_x, insert_y, (node_b + 1)->x, (node_b + 1)->y);
				if (Vehicle::is_node_effect(insert_node_tmp) && (!/*collimap->*/iscollision(node_tmp, insert_node_tmp)) && (!/*collimap->*/iscollision(insert_node_tmp, *(node_b + 1))))
				{
					L_tmp = Vehicle::_L_min(new_theta);
					if (std::min(L_min[2 * i], li) < L_tmp)
					{
						flag = 1;
						L_diff_tmp = L_tmp - std::min(L_min[2 * i], li);
						if (L_diff_tmp < L_diff)
						{
							insert_node = insert_node_tmp;
							L_diff = L_diff_tmp;
						}
					}
					else
					{
						flag = 0;
						insert_node = insert_node_tmp;
						break;
					}
				}
			}
			if (flag == -1)
			{
				if (!Vehicle::is_node_effect(insert_node_tmp))
					cout << "insert_node_tmp isn't effective" << endl;
				if (/*collimap->*/iscollision(node_tmp, insert_node_tmp))
					cout << "the path from node_tmp to insert_node_tmp isn't safe" << endl;
				if (/*collimap->*/iscollision(insert_node_tmp, *(node_b + 1)))
					cout << "the path from insert_node_tmp to *(node_b + 1) isn't safe" << endl;

				flag = 0;
				if (insert_node_set.empty())
				{
					if (node_tmp == *(route.begin() + 1))
					{
						ctrl_points->emplace_back(0.5*(node_tmp.x + (node_b + 1)->x), 0.5*(node_tmp.y + (node_b + 1)->y));
						ctrl_points->emplace_back((node_b + 1)->x, (node_b + 1)->y);
					}
				}
				else
				{
					reference_node = *node_b;
					for (auto n = insert_node_set.rbegin(); n != insert_node_set.rend(); n++)
					{
						ctrl_points->emplace_back(0.5*(reference_node.x + n->second.x), 0.5*(reference_node.y + n->second.y));
						ctrl_points->emplace_back(n->second.x, n->second.y);
						reference_node = n->second;
					}
					ctrl_points->emplace_back(0.5*(reference_node.x + (node_b + 1)->x), 0.5*(reference_node.y + (node_b + 1)->y));
					ctrl_points->emplace_back((node_b + 1)->x, (node_b + 1)->y);
					(node_b + 1)->reset(atan2((node_b + 1)->y - reference_node.y, (node_b + 1)->x - reference_node.x));
					insert_node_set.clear();
				}
				
				lfi = dist(reference_node, *(node_b + 1));
				insert_refer = reference_node = node_tmp;
				node_tmp = *(++node_b);
			}
			else if (flag == 0)
			{
				if (insert_node_set.empty())
				{
					ctrl_points->emplace_back(0.5*(node_tmp.x + insert_node.x), 0.5*(node_tmp.y + insert_node.y));
					ctrl_points->emplace_back(insert_node.x, insert_node.y);
					ctrl_points->emplace_back(0.5*(insert_node.x + (node_b + 1)->x), 0.5*(insert_node.y + (node_b + 1)->y));
					ctrl_points->emplace_back((node_b + 1)->x, (node_b + 1)->y);
					(node_b + 1)->reset(atan2((node_b + 1)->y - insert_node.y, (node_b + 1)->x - insert_node.x));
				}
				else
				{
					if (L_diff < insert_node_set.begin()->first)
						insert_node_set.insert({ L_diff, insert_node });
					reference_node = *node_b;
					for (auto n = insert_node_set.rbegin(); n != insert_node_set.rend(); n++)
					{
						ctrl_points->emplace_back(0.5*(reference_node.x + n->second.x), 0.5*(reference_node.y + n->second.y));
						ctrl_points->emplace_back(n->second.x, n->second.y);
						reference_node = n->second;
					}
					//ctrl_points->emplace_back(0.5*(reference_node.x + node_tmp.x), 0.5*(reference_node.y + node_tmp.y));
					//ctrl_points->emplace_back(node_tmp.x, node_tmp.y);
					ctrl_points->emplace_back(0.5*(reference_node.x + (node_b + 1)->x), 0.5*(reference_node.y + (node_b + 1)->y));
					ctrl_points->emplace_back((node_b + 1)->x, (node_b + 1)->y);
					(node_b + 1)->reset(atan2((node_b + 1)->y - reference_node.y, (node_b + 1)->x - reference_node.x));
					//node_b->reset(atan2(node_b->y - reference_node.y, node_b->x - reference_node.x));
					insert_node_set.clear();
				}

				lfi = dist(reference_node, *(node_b + 1));
				insert_refer = reference_node = node_tmp;
				//node_tmp;
				node_tmp = *(++node_b);
			}
			else
			{
				if (insert_node_set.empty())
				{
					insert_node_set.insert({ L_diff, insert_node });
					reference_node = node_tmp;
					node_tmp = insert_node;
					(node_b + 1)->reset(atan2((node_b + 1)->y - node_tmp.y, (node_b + 1)->x - node_tmp.x));
				}
				else //refer_insert未赋值！！！
				{
					li = Vehicle::_L_min(insert_refer.theta - atan2(insert_node.y - insert_refer.y, insert_node.x - insert_refer.x)) - std::min(lfi, dist(insert_refer, insert_node));
					li += Vehicle::_L_min(atan2(insert_node.y - insert_refer.y, insert_node.x - insert_refer.x) - atan2((node_b + 1)->y - insert_node.y, (node_b + 1)->x - insert_node.x)) - std::min(lb, dist(insert_refer, insert_node));
					if (li < std::min(L_diff, insert_node_set.rbegin()->first))
					{
						insert_node_set.erase(insert_node_set.rbegin()->first);
						insert_node_set.insert({ li, insert_node });
						reference_node = insert_refer;
						node_tmp = insert_node;
						(node_b + 1)->reset(atan2((node_b + 1)->y - node_tmp.y, (node_b + 1)->x - node_tmp.x));
					}
					else if (L_diff < insert_node_set.begin()->first)
					{
						if (insert_node_set.size() == 1)
							insert_refer = node_tmp;
						else
						{
							lfi = dist(insert_refer, insert_node_set.rbegin()->second);
							insert_refer = insert_node_set.rbegin()->second;
						}
						insert_node_set.insert({ L_diff, insert_node });
						reference_node = node_tmp;
						node_tmp = insert_node;
						(node_b + 1)->reset(atan2((node_b + 1)->y - node_tmp.y, (node_b + 1)->x - node_tmp.x));
					}
					else
					{
						reference_node = *node_b;
						for (auto n = insert_node_set.rbegin(); n != insert_node_set.rend(); n++)
						{
							ctrl_points->emplace_back(0.5*(reference_node.x + n->second.x), 0.5*(reference_node.y + n->second.y));
							ctrl_points->emplace_back(n->second.x, n->second.y);
							reference_node = n->second;
						}
						//ctrl_points->emplace_back(0.5*(reference_node.x + node_tmp.x), 0.5*(reference_node.y + node_tmp.y));
						//ctrl_points->emplace_back(node_tmp.x, node_tmp.y);
						ctrl_points->emplace_back(0.5*(reference_node.x + (node_b + 1)->x), 0.5*(reference_node.y + (node_b + 1)->y));
						ctrl_points->emplace_back((node_b + 1)->x, (node_b + 1)->y);
						insert_node_set.clear();

						lfi = dist(reference_node, *(node_b + 1));
						insert_refer = reference_node = node_tmp;
						(node_b + 1)->reset(atan2((node_b + 1)->y - reference_node.y, (node_b + 1)->x - reference_node.x));
						node_tmp = *(++node_b);
						flag = 0;
					}
				}
			}
		}

		else
		{
			flag = 0;
			lfi = dist(reference_node, *(node_b + 1));
			insert_refer = reference_node = node_tmp;
			//node_tmp;
			node_b++;
			node_tmp = *node_b;
			
			if (insert_node_set.empty())
			{
				ctrl_points->emplace_back(0.5*(reference_node.x + node_tmp.x), 0.5*(reference_node.y + node_tmp.y));
				ctrl_points->emplace_back(node_tmp.x, node_tmp.y);
			}
			else
			{
				reference_node = *node_b;
				for (auto n = insert_node_set.rbegin(); n != insert_node_set.rend(); n++)
				{
					ctrl_points->emplace_back(0.5*(reference_node.x + n->second.x), 0.5*(reference_node.y + n->second.y));
					ctrl_points->emplace_back(n->second.x, n->second.y);
					reference_node = n->second;
				}
				ctrl_points->emplace_back(0.5*(reference_node.x + node_tmp.x), 0.5*(reference_node.y + node_tmp.y));
				ctrl_points->emplace_back(node_tmp.x, node_tmp.y);
				insert_node_set.clear();
			}

		}
	}

	ctrl_points->emplace_back(0.5f*(route_tree.rbegin()->x + ctrl_points->rbegin()->x), 0.5f*(route_tree.rbegin()->y + ctrl_points->rbegin()->y));
	ctrl_points->emplace_back(route_tree.rbegin()->x, route_tree.rbegin()->y);
	return;
}

void Trajectory::traj::_bspline()
{
	auto n = ctrl_points->size();
	vector<ts::rational> ctrlp;
	switch (n)
	{
	case 2:
		bspline = ts::BSpline(3, 2, 4, TS_CLAMPED);
		ctrlp = bspline.ctrlp();
		for (vector<position<double>>::size_type i = 0; i < 4; i++)
		{
			ctrlp[2 * i] = ((*ctrl_points)[0].x*(3 - i) + (*ctrl_points)[1].x*i) / 3;
			ctrlp[2 * i + 1] = ((*ctrl_points)[0].y*(3 - i) + (*ctrl_points)[1].y*i) / 3;
		}
		bspline.setCtrlp(ctrlp);
		break;
	case 3:
		bspline = ts::BSpline(3, 2, 4, TS_CLAMPED);
		ctrlp = bspline.ctrlp();
		for (vector<position<double>>::size_type i = 0; i < 4; i++)
		{
			ctrlp[2 * i] = ((*ctrl_points)[0].x*(3 - i) + (*ctrl_points)[2].x*i) / 3;
			ctrlp[2 * i + 1] = ((*ctrl_points)[0].y*(3 - i) + (*ctrl_points)[2].y*i) / 3;
		}
		bspline.setCtrlp(ctrlp);
		break;
	default:
		bspline = ts::BSpline(3, 2, n, TS_CLAMPED);
		ctrlp = bspline.ctrlp();
		for (vector<position<double>>::size_type i = 0; i < n; i++)
		{
			ctrlp[2 * i] = (*ctrl_points)[i].x;
			ctrlp[2 * i + 1] = (*ctrl_points)[i].y;
		}
		bspline.setCtrlp(ctrlp);
		break;
	}
}

void Trajectory::traj::_bspline(const vector<Vehicle::Node> &route_tree)
{
	int N = route_tree.size();
	bspline = ts::BSpline(3, 2, 2 * N - 1, TS_CLAMPED);
	vector<ts::rational> ctrlp = bspline.ctrlp();

	ctrlp[0] = route_tree.begin()->x;				ctrlp[1] = route_tree.begin()->y;
	for (int i = 2; i < ctrlp.size(); i += 4)
	{
		ctrlp[i] = 0.5*(ctrlp[i - 2] + route_tree[(int)(0.25*i + 0.5)].x);
		ctrlp[i + 1] = 0.5*(ctrlp[i - 1] + route_tree[(int)(0.25*i + 0.5)].y);
		ctrlp[i + 2] = route_tree[(int)(0.25*i + 0.5)].x;
		ctrlp[i + 3] = route_tree[(int)(0.25*i + 0.5)].y;
	}
	
	for (int i = 0; i < 4 * N - 2; i += 2)
		ctrl_points->emplace_back(ctrlp[i], ctrlp[i + 1]);
	bspline.setCtrlp(ctrlp);
}

bool Trajectory::traj::_path()
{
	double du = 1 / (POINTS_NUM - 1);

	State old_1 = seg_begin;
	position<double> diff_1, diff_2, old_diff;
	if (entire_traj->size() == 1)
		old_diff.reset(du*cos(bound->begin()->_node()->theta), du*sin(bound->begin()->_node()->theta));
	else
		old_diff.reset(old_1._node()->x - (entire_traj->rbegin() + 1)->_node()->x, old_1._node()->y - (entire_traj->rbegin() + 1)->_node()->y);
	
	double theta, k, s;// , dt, fai;
	vector<ts::rational> result;
	
	for (double u = du; u < 1.; u += du)
	{
		result = bspline.evaluate(u).result();
		diff_1.reset(result[0] - old_1._node()->x, result[1] - old_1._node()->y);
		diff_2.reset(diff_1.x - old_diff.x, diff_1.y - old_diff.y);
		theta = atan2(diff_1.y, diff_1.x);
		k = (diff_1.x*diff_2.y - diff_2.x*diff_1.y) / pow(dist(diff_1), 3);

		/*if (k > KMAX_real) //曲率不满足要求，查找失败，只存0.25个路径长度
		{
			int N = (int)std::floor(entire_traj->size() / 4);
			if ((*entire_traj)[N]._s() < seg_begin._v()*0.1)
				N = 0;

			while (entire_traj->size() > N)
				entire_traj->pop_back();

			return false;
		}*/

		//fai = atan(LA*k);
		s = dist(result[0], result[1], old_1._node()->x, old_1._node()->y);
		//dt = (std::abs(k - old_1._k())*LA*pow(cos(fai), 2)) / DFAIMAX;
		old_1 = State(result[0], result[1], theta, k, s + old_1._s());
//		old_1._t(dt);
		entire_traj->emplace_back(old_1);
		//old_1 = *entire_traj->rbegin();
		old_diff = diff_1;
	}
	s = sqrt(pow(ctrl_points->rbegin()->x - result[0], 2.) + pow(ctrl_points->rbegin()->y - result[1], 2.));
	//fai = atan(LA*seg_end._k());
	//dt = (std::abs(seg_end._k() - old_1._k())*LA*pow(cos(fai), 2)) / DFAIMAX;
	seg_end._s(s + old_1._s());
	//seg_end._t(dt);
	entire_traj->emplace_back(seg_end);
	return true;
}

bool Trajectory::traj::_v_segments(const int u_base)
{
	//u_div->clear();
	//v_div->clear();

//	double accel = seg_begin._a();
	vector<double> v_segments_index; //用于储存分段点的序号
	bool is_success = true;

	int u = 1 + u_base;
	//int N = 2;//在k单调变化的一段曲线内，将曲线分为N段用于分段速度计算
	double dk1 = (*entire_traj)[u]._k() - (*entire_traj)[u - 1]._k();
	double dk2 = dk1;
	double ke = 0;
	for (auto i = entire_traj->begin() + 2 + u_base; i != entire_traj->end() - 1; i++)
	{
		u++;
		dk1 = dk2;
		dk2 = i->_k() - (i - 1)->_k();
		if (dk2*dk1 < 0 && std::abs(i->_k())>eps)
		{
			v_segments_index.push_back(u);
			ke = std::max(i->_k(), ke);
			continue;
		}
	}

	/*if (ke > (KMAX))
		return false;*/

	int N = entire_traj->size() - 1;
	//int n_end = 0.99*N;

	double v0 = seg_begin._v();
	double vg = seg_end._v();
	if (v_segments_index.empty()) //整段路径曲率变化平稳为0
	{
		double s0 = (*entire_traj)[N]._s() - (*entire_traj)[0]._s();
		double v_allow = std::min(sqrt(pow(v0, 2) + 2 * AMAX * s0) - ERROR_V, vg);
		if (sqrt(pow(v0, 2) + 2 * AMIN * s0) > v_allow)
			is_success = false;
		else
		{
			u_div->push_back(N);		v_div->push_back(v_allow);
			u_div->push_back(N);		v_div->push_back(v_allow);
			u_div->push_back(N);		v_div->push_back(v_allow);
		}
	}
	else
	{
		double s0 = (*entire_traj)[*v_segments_index.begin()]._s() - (*entire_traj)[0]._s();
		vector<double> v_candi = { VMAX - ERROR_V, sqrt(ALATER / ke) - ERROR_V, sqrt(pow(v0, 2) + 2 * AMAX * s0) - ERROR_V };
		v_candi.push_back(std::max(V_PRE, v0));
		double v_allow = *std::min_element(v_candi.begin(), v_candi.end());
		if (sqrt(pow(v0, 2) + 2 * AMIN * s0) > v_allow)
			is_success = false;
		else
		{
			u_div->push_back(*v_segments_index.begin());		v_div->push_back(v_allow);
			u_div->push_back(*v_segments_index.rbegin());		v_div->push_back(v_allow);
			u_div->push_back(N);								v_div->push_back(vg);
		}
	}
	return is_success;
}

bool Trajectory::traj::_path(const int &search_result, const vector<Vehicle::Node> &route_tree)
{
	_v_init_goal(search_result);

	vector<Vehicle::Node> route = route_tree;
	orient_trans_vehicle_global(*bound->begin()->_node(), &route);

	switch (search_result)
	{
	case 2:
		_bspline(route);
		break;
	case 0:
		seg_begin = *entire_traj->rbegin();
		seg_end = State(*route.rbegin(), 0, 0);
		_ctrl_points(route);
		_bspline();
		break;
	case 1:
		seg_begin = *entire_traj->rbegin();
		seg_end = *bound->rbegin();
		_ctrl_points(route);
		_bspline();
		break;
	}

	int m = entire_traj->size();

	bool result = _path();

	int n = entire_traj->size();
	
	if (m < n)
		_v_segments(std::max((m - 1), 0));

	return result;
}

void Trajectory::traj::_v_init_goal(const int &is_search_succeed)
{
	if (is_search_succeed==0)
	{
		double now_to_goal = dist(*ctrl_points->begin(), *ctrl_points->rbegin()) / dist(ctrl_points->begin()->x, ctrl_points->begin()->y, bound->rbegin()->_node()->x, bound->rbegin()->_node()->y);
		seg_end._v((1 - now_to_goal)*seg_begin._v() + now_to_goal*bound->rbegin()->_v());
	}
}

void Trajectory::traj::reset(const State &Xi, const State &Xg)
{
	bound->clear();
	entire_traj->clear();

	bound->emplace_back(Xi);
	bound->emplace_back(Xg);
	seg_begin = Xi;
	seg_end = Xg;
	entire_traj->push_back(*bound->begin());
}