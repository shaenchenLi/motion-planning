#include "Trajectory.h"

double eps = 1e-6;

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

void Trajectory::traj::_ctrl_points(const std::vector<Vehicle::Node> &route_tree, vector<double> *L_theta, Collision::collision *collimap)
{
	vector<double> L_min;
	for (auto i = L_theta->rbegin(); i != L_theta->rend(); i++)
		L_min.emplace_back(*i);
	L_min.emplace_back(0);      L_min.emplace_back(PI);

	double x, y, theta;

	std::vector<Vehicle::Node> tree_tmp = route_tree;
	std::vector<Vehicle::Node> route = { *tree_tmp.begin() };
	for (auto node = tree_tmp.begin(); node != tree_tmp.end(); node++)
	{
		if (node == tree_tmp.begin())
		{
			x = tree_tmp.begin()->x + SAFESTEP * cos(tree_tmp.begin()->theta);
			y = tree_tmp.begin()->y + SAFESTEP * sin(tree_tmp.begin()->theta);
			route.emplace_back(x, y, tree_tmp.begin()->theta, 0.f);
			(tree_tmp.begin() + 1)->reset(atan2(node->y - y, node->x - x));
			ctrl_points.emplace_back(0.5*(x + route.begin()->x), 0.5*(y + route.begin()->y));
			ctrl_points.emplace_back(x, y);
			continue;
		}
		else if (node == tree_tmp.end() - 1)
		{
			x = tree_tmp.rbegin()->x - SAFESTEP * cos(tree_tmp.rbegin()->theta);
			y = tree_tmp.rbegin()->y - SAFESTEP * sin(tree_tmp.rbegin()->theta);
			theta = atan2(y - route.rbegin()->y, x - route.rbegin()->x);
			double dis = dist(route.rbegin()->x, route.rbegin()->y, x, y);
			for (double s = SAFESTEP; s < dis; s += SAFESTEP)
			{
				if (collimap->iscollision(route.rbegin()->x + s*cos(theta), route.rbegin()->y + s*sin(theta), theta))
				{
					route.emplace_back(*(node - 1));
					break;
				}
			}
			route.emplace_back(x, y, atan2(y - route.rbegin()->y, x - route.rbegin()->x), 0.f);
			route.emplace_back(*tree_tmp.rbegin());
			continue;
		}

		double dis = dist(*route.rbegin(), *node);
		double theta = atan2(node->y - route.rbegin()->y, node->x - route.rbegin()->x);
		for (double s = SAFESTEP; s < dis; s += SAFESTEP)
		{
			if (collimap->iscollision(route.rbegin()->x + s*cos(theta), route.rbegin()->y + s*sin(theta), theta))
			{
				route.emplace_back(*(node - 1));
				break;
			}
		}
	}

	Vehicle::Node reference_node = *(route.begin());
	int flag = 0; // 0:node++ 1:insert_node 2:end
	auto node = route.begin() + 1;
	Vehicle::Node insert_node_tmp;
	Vehicle::Node node_behind = &*(route.rbegin());
	Vehicle::Node node_tmp = &*node;
	Vehicle::Node insert_node = &*node;
	std::map<double, Vehicle::Node> insert_node_set;

	while (flag != 2)
	{
		if (node == route.end() - 3 || node == route.end() - 2) // the end of the ctrlpoints
		{
			if (flag == 0)
			{
				reference_node = *node;
				node_tmp = &*(++node);
				insert_node_set.clear();
				insert_node_set.insert({ 1000., *(route.end() - 2) });
			}

			double theta = node_tmp.theta - node_behind.theta;
			norm_theta_pi(&theta);
			double L_real = std::min(dist(reference_node, node_tmp), dist(node_tmp, node_behind));
			if (L_real < Vehicle::_L_min(theta))
			{
				flag = -1;
				double L_tmp, L_diff_tmp, L_diff = HUGE_VALD;
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
							} while (L_min[2 * i] > dist(node_tmp, node_behind));
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
					break;
				else if (flag == 2)
				{
					if (L_diff < insert_node_set.begin()->first)
						insert_node_set.insert({ L_diff, insert_node });
					for (auto n = insert_node_set.begin(); n != insert_node_set.end(); n++)
					{
						ctrl_points.emplace_back(0.5*(reference_node.x + n->second.x), 0.5*(reference_node.y + n->second.y));
						ctrl_points.emplace_back(n->second.x, n->second.y);
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
							ctrl_points.emplace_back(0.5*(reference_node.x + n->second.x), 0.5*(reference_node.y + n->second.y));
							ctrl_points.emplace_back(n->second.x, n->second.y);
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
					ctrl_points.emplace_back(0.5f*(reference_node.x + node_tmp.x), 0.5f*(reference_node.y + node_tmp.y));
					ctrl_points.emplace_back(node_tmp.x, node_tmp.y);
				}
				else
				{
					for (auto n = insert_node_set.begin(); n != insert_node_set.end(); n++)
					{
						ctrl_points.emplace_back(0.5*(reference_node.x + n->second.x), 0.5*(reference_node.y + n->second.y));
						ctrl_points.emplace_back(n->second.x, n->second.y);
						reference_node = n->second;
					}
				}
				flag = 2;
				break;
			}
		}

		double theta = node_tmp.theta - (node + 1)->theta;
		norm_theta_pi(&theta);
		double L_real = std::min(dist(reference_node, node_tmp), dist(node_tmp, *(node + 1)));
		if (L_real < Vehicle::_L_min(theta))
		{
			flag = -1;
			double L_tmp, L_diff_tmp, L_diff = HUGE_VALD;
			for (int i = 0; i < 7; i++)
			{
				if (L_min[2 * i] > dist(reference_node, node_tmp))
				{
					if (i == 6)
					{
						do
						{
							node_tmp.reset(node_tmp.x + 0.05*cos(node_tmp.theta), node_tmp.y + 0.05*sin(node_tmp.theta));
							(node + 1)->reset(atan2((node + 1)->y - node_tmp.y, (node + 1)->x - node_tmp.x));
						} while (L_min[2 * i] > dist(reference_node, node_tmp));
					}
					else
						continue;
				}
				double bbbbb = (node + 1)->theta; norm_theta_2pi(&bbbbb);
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
				double new_theta = insert_theta - atan2((node + 1)->y - insert_y, (node + 1)->x - insert_x); // insert node's theta
				norm_theta_pi(&new_theta);
				if (Vehicle::is_node_effect(insert_node_tmp) && (!collimap->iscollision(node_tmp, insert_node_tmp)) && (!collimap->iscollision(insert_node_tmp, *(node + 1))))
				{
					L_tmp = Vehicle::_L_min(new_theta, 0);
					if (std::min(L_min[2 * i], dist(insert_x, insert_y, (node + 1)->x, (node + 1)->y)) < L_tmp)
					{
						flag = 1;
						L_diff_tmp = L_tmp - std::min(L_min[2 * i], dist(insert_x, insert_y, (node + 1)->x, (node + 1)->y));
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
				reference_node = *node;
			}
			else if (flag == 0)
			{
				if (insert_node_set.empty())
				{
					ctrl_points.emplace_back(0.5*(node_tmp.x + insert_node.x), 0.5*(node_tmp.y + insert_node.y));
					ctrl_points.emplace_back(insert_node.x, insert_node.y);
					ctrl_points.emplace_back(0.5*(insert_node.x + (node + 1)->x), 0.5*(insert_node.y + (node + 1)->y));
					ctrl_points.emplace_back((node + 1)->x, (node + 1)->y);
				}
				else
				{
					if (L_diff < insert_node_set.begin()->first)
						insert_node_set.insert({ L_diff, insert_node });
					reference_node = *node;
					for (auto n = insert_node_set.rbegin(); n != insert_node_set.rend(); n++)
					{
						ctrl_points.emplace_back(0.5*(reference_node.x + n->second.x), 0.5*(reference_node.y + n->second.y));
						ctrl_points.emplace_back(n->second.x, n->second.y);
						reference_node = n->second;
					}
					ctrl_points.emplace_back(0.5*(reference_node.x + node_tmp.x), 0.5*(reference_node.y + node_tmp.y));
					ctrl_points.emplace_back(node_tmp.x, node_tmp.y);
					insert_node_set.clear();
				}

				reference_node = node_tmp;
				node_tmp = *(++node);
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
						reference_node = *node;
						for (auto n = insert_node_set.rbegin(); n != insert_node_set.rend(); n++)
						{
							ctrl_points.emplace_back(0.5*(reference_node.x + n->second.x), 0.5*(reference_node.y + n->second.y));
							ctrl_points.emplace_back(n->second.x, n->second.y);
							reference_node = n->second;
						}
						ctrl_points.emplace_back(0.5*(reference_node.x + node_tmp.x), 0.5*(reference_node.y + node_tmp.y));
						ctrl_points.emplace_back(node_tmp.x, node_tmp.y);
						insert_node_set.clear();

						reference_node = node_tmp;
						node_tmp = *(++node);
						flag = 0;
					}
				}
			}
		}

		else
		{
			flag = 0;
			reference_node = node_tmp;
			node++;
			node_tmp = *node;

			if (insert_node_set.empty())
			{
				ctrl_points.emplace_back(0.5*(reference_node.x + node_tmp.x), 0.5*(reference_node.y + node_tmp.y));
				ctrl_points.emplace_back(node_tmp.x, node_tmp.y);
			}
			else
			{
				reference_node = *node;
				for (auto n = insert_node_set.rbegin(); n != insert_node_set.rend(); n++)
				{
					ctrl_points.emplace_back(0.5*(reference_node.x + n->second.x), 0.5*(reference_node.y + n->second.y));
					ctrl_points.emplace_back(n->second.x, n->second.y);
					reference_node = n->second;
				}
				ctrl_points.emplace_back(0.5*(reference_node.x + node_tmp.x), 0.5*(reference_node.y + node_tmp.y));
				ctrl_points.emplace_back(node_tmp.x, node_tmp.y);
				insert_node_set.clear();
			}
		}
	}

	ctrl_points.emplace_back(0.5f*(route_tree.rbegin()->x + ctrl_points.rbegin()->x), 0.5f*(route_tree.rbegin()->y + ctrl_points.rbegin()->y));
	ctrl_points.emplace_back(route_tree.rbegin()->x, route_tree.rbegin()->y);
}

void Trajectory::traj::_bspline()
{

	auto n = ctrl_points.size();

	if (n > 5)
		bspline = ts::BSpline(3, 2, n, TS_CLAMPED);
	else
		bspline = ts::BSpline(n - 3, 2, n, TS_CLAMPED);

	vector<ts::rational> ctrlp = bspline.ctrlp();

	for (vector<Point2D>::size_type i = 0; i < n; i++)
	{
		ctrlp[2 * i] = ctrl_points[i].x;
		ctrlp[2 * i + 1] = ctrl_points[i].y;
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
		ctrl_points.emplace_back(ctrlp[i], ctrlp[i + 1]);
	bspline.setCtrlp(ctrlp);
}

void Trajectory::traj::_state(double *accel, std::vector<State> *adjust_states_end)
{
	double u0[2];
	if (!knots.empty())
	{
		for (auto &i : u0)
		{
			i = *knots.rbegin();
			knots.pop_back();
		}
	}
	else
		state_future = *adjust_states_end;

	double du = 0.001f;
	State old_1, old_2;
	Point2D diff_1, diff_2, old_diff;
	for (auto u = u0[0]; u < u0[1]; u += du)
	{
		if (state_future.empty())
		{
			if (state_now.empty())
			{
				state_future.emplace_back(*bound.begin());
				old_2 = State(bound.begin()->_node()->x - du*cos(bound.begin()->_node()->theta), bound.begin()->_node()->y - du*sin(bound.begin()->_node()->theta));
				old_diff.reset(du*cos(bound.begin()->_node()->theta), du*sin(bound.begin()->_node()->theta));
				continue;
			}
			else
			{
				old_1 = *state_now.rbegin();
				old_2 = *(state_now.rbegin() + 1);
				old_diff.reset(old_1._node()->x - old_2._node()->x, old_1._node()->y - old_2._node()->y);
			}
		}

		auto result = bspline.evaluate(u).result();
		old_1 = *state_future.rbegin();
		diff_1.reset(result[0] - old_1._node()->x, result[1] - old_1._node()->y);
		diff_2.reset(diff_1.x - old_diff.x, diff_1.y - old_diff.y);
		double theta = atan2(diff_1.y, diff_1.x);
		double k = (diff_1.x*diff_2.y - diff_2.x*diff_1.y) / pow(dist(diff_1), 3);
		double s = dist(result[0], result[1], old_1._node()->x, old_1._node()->y);
		state_future.emplace_back(result[0], result[1], theta, k, s + old_1._s_init());
		old_diff = diff_1;
		old_2 = old_1;
	}

	double s = state_future.rbegin()->_s_init() - state_future.begin()->_s_init();
	double vg = *v_tmp_dest.rbegin();
	v_tmp_dest.pop_back();
	Vector4d coeff_v;

	coeff_v[0] = state_now.empty() ? bound.begin()->_v() : state_now.rbegin()->_v();
	coeff_v[1] = *accel;

	double vv = 2 * coeff_v(0) + vg;
	double delta = vv*vv + 6 * coeff_v[1] * s;
	if (delta > -eps)
	{
		if (abs(coeff_v[1]) <= eps)
		{
			if (vv == 0)
			{
				if (coeff_v[1] > eps)
					coeff_v[3] = sqrt(6 * s / coeff_v[1]);
			}
			else
				coeff_v[3] = 3 * s / vv;
		}

		else
			coeff_v[3] = (sqrt(std::max(0., delta)) - vv) / coeff_v[1];
	}

	if (coeff_v[3] > 0)
		coeff_v[2] = (vg - coeff_v[0] - coeff_v[3] * coeff_v[1]) / (coeff_v[3] * coeff_v[3]);
	else
	{
		for (auto &i : state_future)
			i._v(std::min(coeff_v[0], std::min(sqrt(ALATER / abs(i._k())), VMAX)));

		*accel = 0;
		return;
	}

	int N = state_future.size();

	ArrayXXd times = VectorXd::LinSpaced(N, 0., coeff_v[3]);
	ArrayXXd lengths = times*(coeff_v[0] + times*(coeff_v[1] / 2.f + times*coeff_v[2] / 3.f));

	int j = 1;
	double t = 0.f;
	for (auto &i : state_future)
	{
		while (i._s_init() >= lengths(j, 0) && j < N - 1)
			j++;

		t = (times(j - 1, 0) + (i._s_init() - lengths(j - 1, 0))*(times(j, 0) - times(j - 1, 0)) / (lengths(j, 0) - lengths(j - 1, 0)));
		i._v(std::min(coeff_v[0] + coeff_v[1] * t + coeff_v[2] * pow(t, 2), std::min(sqrt(ALATER / abs(i._k())), VMAX)));
	}

	*accel = 2 * coeff_v[0] * t + coeff_v[1];
}

void Trajectory::traj::_v_tmp_dest(const double &accel)
{
	double s = dist(bound[0]._node(), bound[1]._node());
	auto N = knots.size();

	Vector4d coeff_v;

	coeff_v[0] = bound[0]._v();
	coeff_v[1] = accel;

	double vv = 2 * coeff_v(0) + bound[1]._v();
	double delta = vv*vv + 6 * coeff_v[1] * s;
	if (delta > -eps)
	{
		if (abs(coeff_v[1]) <= eps)
		{
			if (vv == 0)
			{
				if (coeff_v[1] > eps)
					coeff_v[3] = sqrt(6 * s / coeff_v[1]);
			}
			else
				coeff_v[3] = 3 * s / vv;
		}

		else
			coeff_v[3] = (sqrt(std::max(0., delta)) - vv) / coeff_v[1];
	}

	if (coeff_v[3] > 0)
		coeff_v[2] = (bound[1]._v() - coeff_v[0] - coeff_v[3] * coeff_v[1]) / (coeff_v[3] * coeff_v[3]);
	else
	{
		for (int i = 1; i <= N / 2 - 1; i++)
			v_tmp_dest.emplace_back(coeff_v[0]);
		return;
	}

	ArrayXXd times = VectorXd::LinSpaced(N, 0., coeff_v[3]);
	ArrayXXd lengths = times*(coeff_v[0] + times*(coeff_v[1] / 2.f + times*coeff_v[2] / 3.f));

	int j = 1;
	double t = 0.;
	for (int i = N - 1; i > 0; i -= 2)
	{
		while (knots[i] * s >= lengths(j, 0) && j < N - 1)
			j++;

		t = (times(j - 1, 0) + (knots[i] * s - lengths(j - 1, 0))*(times(j, 0) - times(j - 1, 0)) / (lengths(j, 0) - lengths(j - 1, 0)) + t);
		v_tmp_dest.push_back(coeff_v[0] + coeff_v[1] * t + coeff_v[2] * pow(t, 2));
	}
	std::reverse(v_tmp_dest.begin(), v_tmp_dest.end());
}

std::vector<Trajectory::State>* Trajectory::traj::_state(const double &a0, const double &sg, std::vector<State> *adjust_states_end)
{
	double accel = a0;
	if (sg < (state_now.cbegin() + 9)->_s_init())
		_state(&accel, adjust_states_end);
	if (sg == (state_now.cbegin() + 1)->_s_init())
	{
		state_now.clear();
		state_now.resize(state_future.size());
		state_now = state_future;
		state_future.clear();
	}
	return &state_now;
}

void Trajectory::adjust_k(const VectorXd &X, State *state, std::vector<State> *adjust_states, const int &flag)
{
	double dk_max = 1 / (LA*pow(cos(WMAX), 2));
	double s_total = X[3] * X[4] * dk_max;
	double s0 = 0.05*X[4];
	//Vehicle::Node node;
	//double v, s_init;
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