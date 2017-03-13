#include "Trajectory.h"

double eps = 1e-6;
Collision::collision* collimap = new Collision::collision();

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

double Trajectory::dist(const Point2D &p)
{
	return sqrt(pow(p.x, 2) + pow(p.y, 2));
}

void Trajectory::traj::_ctrl_points(const std::vector<Vehicle::Node> &route_tree)
{
	vector<Point2D>().swap(*ctrl_points);
	ctrl_points->emplace_back(route_tree.begin()->x, route_tree.begin()->y);
	vector<double> L_min;
	for (auto i = L_theta.rbegin(); i != L_theta.rend();i++)
		L_min.emplace_back(*i);
	L_min.emplace_back(0);      L_min.emplace_back(PI);

	double x, y, theta;

	std::vector<Vehicle::Node> tree_tmp = route_tree;
	std::vector<Vehicle::Node> route = { *tree_tmp.begin() };
	for (auto node_a = tree_tmp.begin(); node_a != tree_tmp.end(); node_a++)
	{
		if (node_a == tree_tmp.begin())
		{
			x = tree_tmp.begin()->x + SAFESTEP * cos(tree_tmp.begin()->theta);
			y = tree_tmp.begin()->y + SAFESTEP * sin(tree_tmp.begin()->theta);
			route.emplace_back(x, y, tree_tmp.begin()->theta, 0.f);
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
				if (collimap->iscollision(route.rbegin()->x + s*cos(theta), route.rbegin()->y + s*sin(theta), theta))
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
			if (collimap->iscollision(route.rbegin()->x + s*cos(theta), route.rbegin()->y + s*sin(theta), theta))
			{
				route.emplace_back(*(node_a - 1));
				break;
			}
		}
	}

	Vehicle::Node reference_node = *(route.begin());
	int flag = 0; // 0:node++ 1:insert_node 2:end
	auto node_b = route.begin();
	node_b++;
	Vehicle::Node insert_node_tmp;
	Vehicle::Node node_behind = &*(route.rbegin());
	Vehicle::Node node_tmp = &*node_b;
	Vehicle::Node insert_node = &*node_b;
	std::map<double, Vehicle::Node> insert_node_set;

	while (flag!=2)
	{
		if (node_b == route.end() - 3 || node_b == route.end() - 2) // the end of the ctrlpoints
		{
			if (flag == 0)
			{
				reference_node = *node_b;
				node_tmp = &*(++node_b);
				insert_node_set.clear();
				insert_node_set.insert({ 1000., *(route.end() - 2) });
			}

			double theta = node_tmp.theta - node_behind.theta;
			norm_theta_pi(&theta);
			double L_real = std::min(dist(reference_node, node_tmp), dist(node_tmp, node_behind));
			if (L_real < Vehicle::_L_min(theta, 0.))
			{
				flag = -1;
				double L_tmp, L_diff_tmp, L_diff = HUGE_VAL;
				for (int i = 0; i < 7; i++)
				{
					if (L_min[2 * i] > dist(node_tmp, node_behind))
					{
						if (i == 6)
						{
							do
							{
								node_tmp.reset(node_tmp.x - 0.05*cos(node_behind.theta), node_tmp.y - 0.05*sin(node_behind.theta));
								node_tmp.reset(atan2(node_tmp.y - reference_node.y, node_tmp.x - reference_node.x));
							}
							while (L_min[2 * i] > dist(node_tmp, node_behind));
						}
						else
							continue;
					}
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
					if (Vehicle::is_node_effect(insert_node_tmp) && (!collimap->iscollision(node_tmp, insert_node_tmp)) && (!collimap->iscollision(insert_node_tmp, node_behind)))
					{
						L_tmp = Vehicle::_L_min(new_theta, 0.);
						if (std::min(L_min[2 * i], dist(insert_x, insert_y, reference_node.x, reference_node.y)) < L_tmp)
						{
							flag = 1;
							L_diff_tmp = L_tmp - std::min(L_min[2 * i], dist(insert_x, insert_y, reference_node.x, reference_node.y));
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
					for (auto n = insert_node_set.begin(); n != insert_node_set.end(); n++)
					{
						ctrl_points->emplace_back(0.5*(reference_node.x + n->second.x), 0.5*(reference_node.y + n->second.y));
						ctrl_points->emplace_back(n->second.x, n->second.y);
						reference_node = n->second;
					}
					break;
				}
				else
				{
					if (L_diff < insert_node_set.begin()->first)
					{
						insert_node_set.insert({ L_diff, insert_node });
						node_behind = node_tmp;
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
				flag = 2;
				break;
			}
		}
				
		double theta = node_tmp.theta - (node_b + 1)->theta;
		norm_theta_pi(&theta);
		double L_real = std::min(dist(reference_node, node_tmp), dist(node_tmp, *(node_b + 1)));
		if (L_real < Vehicle::_L_min(theta, 0.))
		{
			flag = -1;
			double L_tmp, L_diff_tmp, L_diff = HUGE_VALF;
			for (int i = 0; i < 7; i++)
			{
				if (L_min[2 * i] > dist(reference_node, node_tmp))
				{
					if (i == 6)
					{
						do
						{
							node_tmp.reset(node_tmp.x + 0.05*cos(node_tmp.theta), node_tmp.y + 0.05*sin(node_tmp.theta));
							(node_b + 1)->reset(atan2((node_b + 1)->y - node_tmp.y, (node_b + 1)->x - node_tmp.x));
						}
						while (L_min[2 * i] > dist(reference_node, node_tmp));						
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
				if (Vehicle::is_node_effect(insert_node_tmp) && (!collimap->iscollision(node_tmp, insert_node_tmp)) && (!collimap->iscollision(insert_node_tmp, *(node_b + 1))))
				{
					L_tmp = Vehicle::_L_min(new_theta, 0);
					if (std::min(L_min[2 * i], dist(insert_x, insert_y, (node_b + 1)->x, (node_b + 1)->y)) < L_tmp)
					{
						flag = 1;
						L_diff_tmp = L_tmp - std::min(L_min[2 * i], dist(insert_x, insert_y, (node_b + 1)->x, (node_b + 1)->y));
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
				flag = 0;
				reference_node = node_tmp;
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
					ctrl_points->emplace_back(0.5*(reference_node.x + node_tmp.x), 0.5*(reference_node.y + node_tmp.y));
					ctrl_points->emplace_back(node_tmp.x, node_tmp.y);
					insert_node_set.clear();
				}

				reference_node = node_tmp;
				node_tmp = *(++node_b);
			}
			else
			{
				if (insert_node_set.empty())
				{
					insert_node_set.insert({ L_diff, insert_node });
					reference_node = node_tmp;
					node_tmp = insert_node;
				}
				else
				{
					if (L_diff < insert_node_set.begin()->first)
					{
						insert_node_set.insert({ L_diff, insert_node });
						reference_node = node_tmp;
						node_tmp = insert_node;
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

						reference_node = node_tmp;
						node_tmp = *(++node_b);
						flag = 0;
					}
				}
			}
		}

		else
		{
			flag = 0;
			reference_node = node_tmp;
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
}

void Trajectory::traj::_bspline()
{
	auto n = ctrl_points->size();
	bspline = ts::BSpline(3, 2, n, TS_CLAMPED);

	vector<ts::rational> ctrlp = bspline.ctrlp();

	for (vector<Point2D>::size_type i = 0; i < n; i++)
	{
		ctrlp[2 * i] = (*ctrl_points)[i].x;
		ctrlp[2 * i + 1] = (*ctrl_points)[i].y;
	}
	bspline.setCtrlp(ctrlp);
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

void Trajectory::traj::_path()
{
	State old_1 = seg_begin;
	Point2D diff_1, diff_2, old_diff;
	if (state_now->size() == 1)
		old_diff.reset(DU*cos(bound->begin()->_node()->theta), DU*sin(bound->begin()->_node()->theta));
	else
		old_diff.reset(old_1._node()->x - (state_now->rbegin() + 1)->_node()->x, old_1._node()->y - (state_now->rbegin() + 1)->_node()->y);
	
	double theta, k, s, dt, fai;
	vector<ts::rational> result;
	for (double u = DU; u <= 1.; u += DU)
	{
		result = bspline.evaluate(u).result();
		diff_1.reset(result[0] - old_1._node()->x, result[1] - old_1._node()->y);
		diff_2.reset(diff_1.x - old_diff.x, diff_1.y - old_diff.y);
		theta = atan2(diff_1.y, diff_1.x);
		k = (diff_1.x*diff_2.y - diff_2.x*diff_1.y) / pow(dist(diff_1), 3);
		fai = atan(LA*k);
		s = dist(result[0], result[1], old_1._node()->x, old_1._node()->y);
		dt = (std::abs(k - old_1._k())*LA*pow(cos(fai), 2)) / DFAIMAX;
		old_1 = State(result[0], result[1], theta, k, s + old_1._s_init());
		old_1._t(dt);
		state_future->emplace_back(old_1);
		old_diff = diff_1;
	}
	s = sqrt(pow(ctrl_points->rbegin()->x - result[0], 2.) + pow(ctrl_points->rbegin()->y - result[1], 2.));
	fai = atan(LA*seg_end._k());
	dt = (std::abs(seg_end._k() - old_1._k())*LA*pow(cos(fai), 2)) / DFAIMAX;
	seg_end._s_init(s + old_1._s_init());
	seg_end._t(dt);
	state_future->emplace_back(seg_end);
}

bool Trajectory::traj::_state(const double &accel)
{
	vector<double>* v_segments_index = new vector<double>({ 0 });//用于储存分段点的序号
	bool is_success = true;

	int flag = 1; //1表示插入方式为k处于极值，2表示插入方式为k为0
	int u = 1;
	double dk1 = (*state_future)[1]._k() - (*state_future)[0]._k();
	double dk2 = dk1;
	for (auto i = state_future->begin() + 2; i != state_future->end() - 1; i++)
	{
		u++;
		dk1 = dk2;
		dk2 = i->_k() - (i - 1)->_k();
		if (dk2*dk1 < 0)
		{
			v_segments_index->push_back(u);
			flag = 1;
			continue;
		}
		if (i->_k() == 0 || i->_k()*(i - 1)->_k() < 0)
		{
			if (flag == 2)
			{
				v_segments_index->pop_back();
				v_segments_index->push_back(u);
			}
			else
				v_segments_index->push_back(u);
			flag = 2;
		}
	}
	v_segments_index->emplace_back(990);

	//判断是否能进行速度生成，若不能说明路径规划有问题需要重新规划
	double v0 = seg_begin._v();
	double k = std::abs((*state_future)[(*v_segments_index)[1]]._k());
	double s = (*state_future)[(*v_segments_index)[1]]._s_init() - seg_begin._s_init();
	double v_amin = sqrt(pow(v0, 2) + 2 * AMIN * s);
	vector<double>* v_candidate = new vector<double>;// ({ v_ay, v_w, v_amax });
	v_candidate->emplace_back(VMAX);
	v_candidate->emplace_back(sqrt(ALATER / k));
	v_candidate->emplace_back(WMAX / k);
	v_candidate->emplace_back(sqrt(pow(v0, 2) + 2 * AMAX * s));
	if (v_amin > *std::min_element(v_candidate->begin(), v_candidate->end()))
		is_success = false;

	//通过规则确定每段生成速度的目标速度，已经相应点在轨迹中的位置
	else
	{		
		double v_allow, v, a, ds, dt;// , v_mid;
		double k1, k2;
		double a0 = accel;
		int u = 1;
		for (auto i = v_segments_index->begin() + 1; i != v_segments_index->end(); i++)
		{
			if (i == v_segments_index->end() - 1)
				v = seg_end._v();
			else
			{
				k = std::abs((*state_future)[*i]._k());
				if (k != 0)
				{
					v_candidate->clear();
					s = (*state_future)[*i]._s_init() - (*state_future)[*(i - 1)]._s_init();
					v_candidate->emplace_back(VMAX);
					v_candidate->emplace_back(sqrt(ALATER / k));
					v_candidate->emplace_back(WMAX / k);
					v_candidate->emplace_back(sqrt(pow(v0, 2) + 2 * AMAX * s));
					v_allow = *std::min_element(v_candidate->begin(), v_candidate->end());
					v = v_allow;
				}
				else
				{
					if (i == v_segments_index->end() - 1)
						break;
					else
					{
						k = std::abs((*state_future)[*i + 1]._k());
						s = (*state_future)[*i + 1]._s_init() - (*state_future)[*i - 1]._s_init();
						v_candidate->emplace_back(VMAX);
						v_candidate->emplace_back(sqrt(ALATER / k));
						v_candidate->emplace_back(WMAX / k);
						v_candidate->emplace_back(sqrt(pow(v0, 2) + 2 * AMAX * s));
						v_allow = *std::min_element(v_candidate->begin(), v_candidate->end());
						v_amin = sqrt(pow(v0, 2) + 2 * AMIN * s);
						if (v_amin < v_allow)
						{
							is_success = false;
							break;
						}
						v = 0.5*(v_allow + v0);
					}
				}
			}

			double seg_s = (*state_future)[*(i - 1)]._s_init();
			s = (*state_future)[*i]._s_init() - seg_s;
			k1 = a0;
			k2 = (pow(v, 2) - pow(v0, 2)) / pow(s, 2) - 2 * k1 / s;
			a0 = k1 + k2*s;
			if (a0<AMIN || a0>AMAX)
			{
				a0 = a0 > 0 ? AMAX : AMIN;
				k2 = (a0 - k1) / s;
			}

			for (; u <= *i; u++)
			{
				s = (*state_future)[u]._s_init() - seg_s;
				v = pow(v0, 2) + 2 * k1*s + k2*pow(s, 2);
				if (v < 0)
				{
					v = (*state_future)[u - 1]._v();
					a = 0;
				}
				else
				{
					v = sqrt(v);
					a = k1 + k2*s;
				}
				ds = (*state_future)[u]._s_init() - (*state_future)[u - 1]._s_init();
				dt = 2 * ds / (v + (*state_future)[u - 1]._v());
				if (dt < (*state_future)[u]._t())
				{
					v = 2 * ds / (*state_future)[u]._t() - (*state_future)[u - 1]._v();
					(*state_future)[u]._v(v);
					(*state_future)[u]._a((v - (*state_future)[u - 1]._v()) / (*state_future)[u]._t());
				}
				else
				{
					(*state_future)[u]._v(v);
					(*state_future)[u]._a(a);
					(*state_future)[u]._t(dt);
				}
			}
			v0 = (*state_future)[u - 1]._v();
		}

		for (; u != 1000; u++)
			(*state_future)[u]._v(seg_end._v());
		state_future->rbegin()->_a(0);
	}

	if (is_success==false)
	{
		state_future->clear();
		vector<State>* state_future_tmp = new vector<State>;
		double dt = 0.05;
		int i = 1;
		double t, x, y, v, s;
		double s_max = pow(seg_begin._v(), 2) / 2 / abs(AMIN);
		for (auto i = state_future->begin(); i != state_future->end(); i++)
		{
			s = i->_s_init() - seg_begin._s_init();
			if (s >= s_max)
			{
				State tmp(i->_node(), 0, i->_s_init());
				state_future_tmp->emplace_back(tmp);
				break;
			}
			State tmp(*i);
			tmp._v(sqrt(pow(seg_begin._v(), 2) - 2 * AMIN*s));
			tmp._a(AMIN);
			tmp._t((tmp._v() - seg_begin._v()) / AMIN);
			state_future_tmp->emplace_back(tmp);
		}
		state_future->resize(state_future_tmp->size());
		memcpy(&(*state_future)[0], &state_future_tmp[0], state_future_tmp->size()*sizeof(Trajectory::State));
		return false;
	}
	delete v_segments_index;
	return is_success;
}

void Trajectory::traj::_v_init_goal(double *accel, const int &is_search_succeed)
{
	if (state_now->size()!=1)
		*accel = (pow(seg_begin._v(), 2) - pow((state_now->rbegin() + 1)->_v(), 2)) / (2 * (seg_begin._s_init() - (state_now->rbegin() + 1)->_s_init()));
		
	if (is_search_succeed==0)
	{
		double now_to_goal = dist(*ctrl_points->begin(), *ctrl_points->rbegin()) / dist(ctrl_points->begin()->x, ctrl_points->begin()->y, bound->rbegin()->_node()->x, bound->rbegin()->_node()->y);
		seg_end._v((1 - now_to_goal)*seg_begin._v() + now_to_goal*bound->rbegin()->_v());
	}
}

// 在最后阶段如果有最后调整曲率的路线需要转换
void Trajectory::traj::_state(const int &search_result, const double &a0, const vector<Vehicle::Node> &route_tree, vector<State> *adjust_states_end)
{
	double accel = a0;
	vector<double>* v_init_goal = new vector<double>(2);
	switch (search_result)
	{
	case 2:
		_bspline(route_tree);
		break;
	case 0: 	
		seg_begin = *state_now->rbegin();
		seg_end = State(*route_tree.rbegin(), 0, 0);
	case 1: 
		seg_begin = *state_now->rbegin();
		seg_end = *bound->rbegin();
		_ctrl_points(route_tree);
		_bspline();
		break;

	}
	
	_path();
	_v_init_goal(&accel, search_result);
	bool is_success = _state(accel);
	if (search_result != 0 && adjust_states_end != nullptr)
	{
		int n = state_future->size();
		state_future->resize(adjust_states_end->size() + n);
		memcpy(&(*state_future)[n], &(*adjust_states_end), state_future->size()*sizeof(State));
	}
	if (state_now->empty())
	{
		state_now->resize(state_future->size()); 
		memcpy(&(*state_now)[0], &(*state_future)[0], state_future->size()*sizeof(State));
	}
	else
	{
		// 此时路径已经传送到下层控制，因此state_now可以清空
		state_now->clear();
		state_now->resize(state_future->size());
		memcpy(&(*state_now)[0], &(*state_future)[0], state_future->size()*sizeof(State));
	}
	delete v_init_goal;
	vector<State>().swap(*state_future);
}

void Trajectory::adjust_k(const VectorXd &X, State *state, std::vector<State> *adjust_states, const int &flag)
{
	double dk_max = 1 / (LA*pow(cos(WMAX), 2));
	double s_total = X[3]*X[4]*dk_max;
	double s0 = 0.05*X[4];
	double a = X[3];
	double b = -1 / (X[4] * dk_max); //k(s)=a+b*s
	adjust_states->emplace_back(X);
	double x, y, dx, dy;

	for (double s = s0; s < s_total; s += s0)
	{
		dx = cos(X[2] + a) + cos(X[2] + a + b*s / 8);
		dy = sin(X[2] + a) + sin(X[2] + a + b*s / 8);
		for (int i = 1; i <= 5; i += 2)
		{
			dx += 4 * cos(X[2] + a + b*i*s / 8);
			dy += 4 * sin(X[2] + a + b*i*s / 8);
		}
		for (int i = 1; i <= 3; i++)
		{
			dx += 2 * cos(X[2] + a + b*i*s / 4);
			dy += 2 * sin(X[2] + a + b*i*s / 4);
		}
		dx *= s / 24;
		dy *= s / 24;

		double theta = X[2] + a*s + 0.5f*pow(s, 2)*b;
		x = X[0] + dx;
		y = X[1] + dy;
		adjust_states->emplace_back(Vehicle::Node(x, y, theta, a + b*s), X[4], s);
	}

	dx = cos(X[2] + a) + cos(X[2] + a + b*s_total / 8);
	dy = sin(X[2] + a) + sin(X[2] + a + b*s_total / 8);
	for (int i = 1; i <= 5; i += 2)
	{
		dx += 4 * cos(X[2] + a + b*i*s_total / 8);
		dy += 4 * sin(X[2] + a + b*i*s_total / 8);
	}
	for (int i = 1; i <= 3; i++)
	{
		dx += 2 * cos(X[2] + a + b*i*s_total / 4);
		dy += 2 * sin(X[2] + a + b*i*s_total / 4);
	}
	dx *= s_total / 24;
	dy *= s_total / 24;

	x = X[0] + dx;
	y = X[1] + dy;
	state->_node(x, y, X[2] + a*s_total + 0.5f*pow(s_total, 2)*b, a + b*s_total);
	state->_v(X[4]);

	if (flag == 1)
		std::reverse(adjust_states->begin(), adjust_states->end());
}