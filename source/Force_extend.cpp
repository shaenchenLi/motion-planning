#include "Force_extend.h"

int Trajectory::orient_trans(const Vehicle::Node &global_start_node, const Vehicle::Node &global_end_node, const double &angle, Vehicle::Node *local_start_node, Vehicle::Node *local_end_node)
{	
	local_start_node->x = 0;
	local_start_node->y = 0;
	local_start_node->theta = global_start_node.theta - angle;

	local_end_node->theta = global_end_node.theta - angle;
	double xg = global_end_node.x - global_start_node.x;
	double yg = global_end_node.y - global_start_node.y;
	//全局路径在大地坐标系的朝向角为顺时针为正
	local_end_node->x = xg*cos(angle) + yg*sin(angle);
	local_end_node->y = -xg*sin(angle) + yg*cos(angle);
	/*end_node.x = xg*cos(angle) + yg*sin(angle);
	end_node.y = -xg*sin(angle) + yg*cos(angle);*/
	//向右避障为正，如果决策决定向左避障，那么需要将目标点对称为向右避障，同时两个点的theta都要变动
	int is_right = 1;
	if (local_end_node->y < local_start_node->y)
	{
		is_right = -1;
		local_end_node->y = -local_end_node->y;
		local_end_node->theta = -local_end_node->theta;
		local_start_node->theta = -local_start_node->theta;
	}
	return is_right;
}

bool Trajectory::straight(const Vehicle::Node &start_node, const Vehicle::Node &end_node, vector<double> *control)
{
	control->push_back(start_node.x);						  control->push_back(start_node.y);
	control->push_back((2 * start_node.x + end_node.x) / 3);  control->push_back((2 * start_node.y + end_node.y) / 3);
	control->push_back((start_node.x + 2 * end_node.x) / 3);  control->push_back((start_node.y + 2 * end_node.y) / 3);
	control->push_back(end_node.x);							  control->push_back(end_node.y);
	return true;
}

bool Trajectory::lanechange(const Vehicle::Node &start_node, const Vehicle::Node &end_node, const double &angle, vector<double> *control)
{
	control->clear();

	Vehicle::Node local_start_node, local_end_node;
	orient_trans(start_node, end_node, angle, &local_start_node, &local_end_node);
	double l_max = local_end_node.x - local_start_node.x;//允许的l最大值

	double x1_tmp, y1_tmp, x2_tmp, y2_tmp, x1, y1, x2, y2;
	double /*l1, l2, l3, */alfa_tmp, beita_tmp, l1_tmp, l2_tmp, l3_tmp;
	double t;
	bool result = false;
	vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Matrix<double, 6, 1>>> opt;

	int N = (int)(L_theta.size() / 2);
	for (auto a = L_theta.begin(); a != L_theta.begin() + N; a += 2)
	{
		// calculate l1
		alfa_tmp = *a;
		l1_tmp = Vehicle::_L_min(alfa_tmp, 2 * ERROR_KMAX);

		if (l1_tmp >= l_max)
			continue;

		x1_tmp = local_start_node.x + l1_tmp*cos(local_start_node.theta);
		y1_tmp = local_start_node.y + l1_tmp*sin(local_start_node.theta);

		// calculate l2&l3

		t = alfa_tmp - local_start_node.theta;
		beita_tmp = local_end_node.theta + t;

		if (sin(beita_tmp) != 0)
		{
			l2_tmp = ((local_end_node.y - y1_tmp)*cos(local_end_node.theta) - (local_end_node.x - x1_tmp)*sin(local_end_node.theta)) / sin(beita_tmp);
			if (cos(local_end_node.theta) != 0)
				l3_tmp = (local_end_node.x - x1_tmp + l2_tmp*cos(t)) / cos(local_end_node.theta);
			else
				l3_tmp = (local_end_node.y - y1_tmp - l2_tmp*sin(t)) / sin(local_end_node.theta);
		}
		else
		{
			l3_tmp = ((local_end_node.x - x1_tmp)*sin(t) + (local_end_node.y - y1_tmp)*cos(t)) / (2 * sin(beita_tmp));
			if (cos(t) != 0)
				l2_tmp = (x1_tmp - local_end_node.x + l3_tmp*cos(local_end_node.theta)) / cos(t);
			else
				l2_tmp = (local_end_node.y - y1_tmp - l3_tmp*sin(local_end_node.theta)) / sin(t);
		}
		
		if (l3_tmp < 0 || l2_tmp < 0)
			continue;

		//cout << std::min(l2_tmp, l1_tmp) << " " << (std::min(l2_tmp, l3_tmp)) << endl;
		//cout << (std::min(l2_tmp, l1_tmp) < *(a + 1)) << endl;
		//cout << (std::min(l2_tmp, l3_tmp) < _L_min(alfa_tmp + thetag)) << endl;
		//cout << *(a + 1) << " " << _L_min(alfa_tmp + thetag) << endl;

		if (beita_tmp > PI)
			beita_tmp = 2 * PI - beita_tmp;
		if (std::min(l2_tmp, l1_tmp) < *(a + 1) || std::min(l2_tmp, l3_tmp) < Vehicle::_L_min(beita_tmp) || beita_tmp < ALFA_MIN)
			continue;

		//cout << "l2 meets demand" << endl;
		//cout << "l3 meets demand" << endl;

		result = true;
		Matrix<double, 6, 1> opt_data;
		opt_data << std::min({ l1_tmp + l2_tmp + l3_tmp }), l1_tmp, l2_tmp, l3_tmp, alfa_tmp, beita_tmp;
		opt.push_back(opt_data);
	}
	auto min_i = opt.begin();
	double min = 1000;
	if (result == true)
	{
		auto i = opt.begin();
		for (; i != opt.end(); i++)
		{
			if (min > (*i)[0])
			{
				min_i = i;
				min = (*i)[0];
			}
		}

		cout << "l1:" << (*min_i)[1] << " l2:" << (*min_i)[2] << " l3:" << (*min_i)[3] << " alfa:" << (*min_i)[4] << " beita" << (*min_i)[5] << endl;
		x1 = start_node.x + (*min_i)[1] * cos(start_node.theta);      y1 = start_node.y + (*min_i)[1] * sin(start_node.theta);
		x2 = end_node.x - (*min_i)[3] * cos(end_node.theta);		  y2 = end_node.y - (*min_i)[3] * sin(end_node.theta);
		control->push_back(start_node.x); control->push_back(start_node.y);
		control->push_back(x1); control->push_back(y1);
		control->push_back(x2); control->push_back(y2);
		control->push_back(end_node.x); control->push_back(end_node.y);
	}

	return result;
}

bool Trajectory::lanechange_c(const Vehicle::Node &start_node, const Vehicle::Node &end_node, const double &angle, const vector<double> &constraints, vector<double> *control)
{
	control->clear();

	Vehicle::Node local_start_node, local_end_node;
	orient_trans(start_node, end_node, angle, &local_start_node, &local_end_node);
	double l_max = local_end_node.x - local_start_node.x;//允许的l最大值

	double x0 = local_start_node.x;				double y0 = local_start_node.y;
	double x3 = local_end_node.x;				double y3 = local_end_node.y;
	double theta0 = local_start_node.theta;		double theta3 = local_end_node.theta;

	double x1_tmp, y1_tmp, x2_tmp, y2_tmp, x1, y1, x2, y2;
	double /*l1, l2, l3, */alfa_tmp, beita_tmp, l1_tmp, l2_tmp, l3_tmp;
	double t, c;
	bool result = false;
	vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Matrix<double, 6, 1>>> opt;

	double obs_l = constraints[0];
	double obs_d = constraints[1];//当障碍物突出方向和目标点同向时为正，即其符号取决于目标点的位置
	double obs_v = constraints[2]; //障碍物绝对车速
	//cout << bound_condition[0] << " " << bound_condition[1] << " " << bound_condition[2] << endl;
	//cout << "l:" << l << " w:" << w << endl;

	int N = (int)(L_theta.size() / 2);

	double v = sqrt(ALATER / KMAX);

	for (auto a = L_theta.begin(); a != L_theta.begin() + N; a += 2)
	{
		alfa_tmp = *a;
		l1_tmp = Vehicle::_L_min(alfa_tmp, 2 * ERROR_KMAX);

		if (l1_tmp >= l_max)
			continue;

		x1_tmp = x0 + l1_tmp*cos(theta0);
		y1_tmp = y0 + l1_tmp*sin(theta0);

		// calculate l2&l3

		t = alfa_tmp - theta0;
		beita_tmp = theta3 + t;

		if (sin(beita_tmp) != 0)
		{
			l2_tmp = ((y3 - y1_tmp)*cos(theta3) - (x3 - x1_tmp)*sin(theta3)) / sin(beita_tmp);
			if (cos(theta3) != 0)
				l3_tmp = (x3 - x1_tmp + l2_tmp*cos(t)) / cos(theta3);
			else
				l3_tmp = (y3 - y1_tmp - l2_tmp*sin(t)) / sin(theta3);
		}
		else
		{
			l3_tmp = ((x3 - x1_tmp)*sin(t) + (y3 - y1_tmp)*cos(t)) / (2 * sin(beita_tmp));
			if (cos(t) != 0)
				l2_tmp = (x1_tmp - x3 + l3_tmp*cos(theta3)) / cos(t);
			else
				l2_tmp = (y3 - y1_tmp - l3_tmp*sin(theta3)) / sin(t);
		}

		if (l3_tmp < 0 || l2_tmp < 0)
			continue;

		//cout << std::min(l2_tmp, l1_tmp) << " " << (std::min(l2_tmp, l3_tmp)) << endl;
		//cout << (std::min(l2_tmp, l1_tmp) < *(a + 1)) << endl;
		//cout << (std::min(l2_tmp, l3_tmp) < _L_min(alfa_tmp + thetag)) << endl;
		//cout << *(a + 1) << " " << _L_min(alfa_tmp + thetag) << endl;

		if (beita_tmp > PI)
			beita_tmp = 2 * PI - beita_tmp;
		if (std::min(l2_tmp, l1_tmp) < *(a + 1) || std::min(l2_tmp, l3_tmp) < Vehicle::_L_min(beita_tmp) || beita_tmp < ALFA_MIN)
			continue;

		//cout << "l2 meets demand" << endl;
		//cout << "l3 meets demand" << endl;

		x2_tmp = x3 - l3_tmp*cos(theta3);
		y2_tmp = y3 - l3_tmp*sin(theta3);

		double t1 = l1_tmp / v;
		double t2 = (l1_tmp + l2_tmp) / v;

		if (sin(theta0) != 0)
		{
			double ld1 = (obs_d + 0.5*W*abs(cos(theta0))) / sin(theta0) - L_F_BA;
			double ld2 = (obs_d + 0.5*W*abs(cos(theta0))) / sin(theta0) + L - L_F_BA;
			double td1 = ld1 / v;
			double td2 = ld2 / v;
			if (l1_tmp < ld1)
				c = -(ERROR_OBS - (obs_l - L_F_BA*abs(cos(theta0)) - 0.5*W*abs(sin(theta0)) + obs_v*t1 - l1_tmp*abs(cos(theta0))));
			else
			{
				double G4_1 = obs_l + obs_v*td1 - (ld1*abs(cos(theta0)) + 0.5*W*abs(sin(theta0)) + L_F_BA*abs(cos(theta0)));
				double G4_2 = obs_l + obs_v*td1 - 0.5*W / abs(sin(theta0)) - obs_d / abs(tan(theta0));
				G4_2 *= abs(sin(theta0));
				double G4_3;
				if (l1_tmp < ld2)
					G4_3 = obs_l + obs_v*t1 - 0.5*W / abs(sin(theta0)) - obs_d / abs(tan(theta0));
				else
					G4_3 = obs_l + obs_v*td2 - 0.5*W / abs(sin(theta0)) - obs_d / abs(tan(theta0));
				G4_3 *= abs(sin(theta0));
				c = std::min(G4_1, std::min(G4_2, G4_3)) - ERROR_OBS;
			}
		}
		else
		{
			if (W / 2 + obs_d + ERROR_OBS < 0)
				c = 1;
			else
				c = -(ERROR_OBS - (obs_l + obs_v*t1 - (l1_tmp + L_F_BA)));
		}
		if (c <= 0)
			continue;

		double beita_1 = atan2(y2_tmp - y1_tmp, x2_tmp - x1_tmp);
		if (sin(beita_1) != 0)
		{
			double ld4 = (obs_d + 0.5*W*abs(cos(beita_1)) - l1_tmp*sin(theta0)) / sin(beita_1) - L_F_BA;
			double ld5 = (obs_d + 0.5*W*abs(cos(beita_1)) - l1_tmp*sin(theta0)) / sin(beita_1) + L - L_F_BA;
			double td4 = (l1_tmp + ld4) / v;
			double td5 = (l1_tmp + ld5) / v;
			if (l2_tmp < ld4)
				c = obs_l + obs_v*t2 - (l1_tmp*std::abs(cos(theta0)) + (l2_tmp + L_F_BA)*std::abs(cos(beita_1)) + 0.5*W*std::abs(sin(beita_1))) - ERROR_OBS;
			else
			{
				double G5_1 = obs_l + obs_v*t2 - (l1_tmp*std::abs(cos(theta0)) + (ld4 + L_F_BA)*std::abs(cos(beita_1)) + 0.5*W*std::abs(sin(beita_1)));
				double G5_2 = obs_l + obs_v*td4 - l1_tmp*std::abs(cos(theta0)) - 0.5*W / abs(sin(beita_1)) - obs_d / abs(tan(beita_1));
				G5_2 *= std::abs(sin(beita_1));
				double G5_3;
				if (l2_tmp < ld5)
					G5_3 = obs_l + obs_v*t2 - l1_tmp*std::abs(cos(theta0)) - 0.5*W / abs(sin(beita_1)) - obs_d / abs(tan(beita_1));
				else
					G5_3 = obs_l + obs_v*td5 - l1_tmp*std::abs(cos(theta0)) - 0.5*W / abs(sin(beita_1)) - obs_d / abs(tan(beita_1));
				G5_3 *= std::abs(sin(beita_1));
				c = std::min(G5_1, std::min(G5_2, G5_3)) - ERROR_OBS;
			}
		}
		else
		{
			if ((l1_tmp*abs(sin(theta0)) + L_F_BA*sin(beita_1) - 0.5*W*abs(cos(beita_1))) <= obs_d + ERROR_OBS)
				c = -(ERROR_OBS - obs_l - obs_v*t2 + (l1_tmp*abs(cos(theta0)) + v*t2 + L_F_BA));
			else
				c = 1;
		}
		if (c <= 0)
			continue;

		if (sin(theta3) == 0)
		{
			if (y3 > obs_d + ERROR_OBS + 0.5*W*abs(cos(theta3)))
				c = 1;
			else
			{
				double G6_1 = obs_l + obs_v*t2 - (x2_tmp - x0) - ERROR_OBS - L_F_BA;
				double G6_2 = obs_l + obs_v*(l1_tmp + l2_tmp + l3_tmp) / v - (x3 - x0) - ERROR_OBS - L_F_BA;
				c = std::min(G6_1, G6_2); //如果满足不了障碍物在车侧面安全距离，则要求车辆一定在障碍物后方
			}
		}
		else
			c = -(l1_tmp*abs(cos(theta0)) + l2_tmp*abs(cos(beita_1)) + 0.5*W / abs(sin(theta3)) + (obs_d - l1_tmp*sin(theta0) - l2_tmp*sin(beita_1)) / abs(tan(theta3)) - (obs_l + obs_v*t2 + ERROR_OBS));
		if (c <= 0)
			continue;

		result = true;
		Matrix<double, 6, 1> opt_data;
		opt_data << std::min({ l1_tmp + l2_tmp + l3_tmp }), l1_tmp, l2_tmp, l3_tmp, alfa_tmp, beita_tmp;
		opt.push_back(opt_data);
	}
	auto min_i = opt.begin();
	double min = 1000;
	if (result == true)
	{
		//cout << "l1:" << l1 << " l2:" << l2 << " l3:" << l3 << " alfa:" << alfa << endl;
		auto i = opt.begin();
		for (; i != opt.end(); i++)
		{
			if (min > (*i)[0])
			{
				min_i = i;
				min = (*i)[0];
			}
		}

		cout << "l1:" << (*min_i)[1] << " l2:" << (*min_i)[2] << " l3:" << (*min_i)[3] << " alfa:" << (*min_i)[4] << " beita" << (*min_i)[5] << endl;

		x1 = start_node.x + (*min_i)[1] * cos(start_node.theta);      y1 = start_node.y + (*min_i)[1] * sin(start_node.theta);
		x2 = end_node.x - (*min_i)[3] * cos(end_node.theta);		  y2 = end_node.y - (*min_i)[3] * sin(end_node.theta);
		control->push_back(start_node.x); control->push_back(start_node.y);
		control->push_back(x1); control->push_back(y1);
		control->push_back(x2); control->push_back(y2);
		control->push_back(end_node.x); control->push_back(end_node.y);
	}

	return result;
}

bool Trajectory::lanechange_r_c(const Vehicle::Node &start_node, const Vehicle::Node &end_node, const double &angle, const vector<double> &constraints, vector<double> *control)
{
	control->clear();

	Vehicle::Node local_start_node, local_end_node;
	orient_trans(start_node, end_node, angle, &local_start_node, &local_end_node);
	double l_max = local_end_node.x - local_start_node.x;//允许的l最大值

	double x0 = local_start_node.x;				double y0 = local_start_node.y;
	double x3 = local_end_node.x;				double y3 = local_end_node.y;
	double theta0 = local_start_node.theta;		double theta3 = local_end_node.theta;

	double x1_tmp, y1_tmp, x2_tmp, y2_tmp, x1, y1, x2, y2;
	double /*l1, l2, l3,*/ alfa_tmp, beita_tmp, l1_tmp, l2_tmp, l3_tmp;
	double t, c;
	bool result = false;
	vector<Eigen::Matrix<double, 4, 1>, Eigen::aligned_allocator<Matrix<double, 4, 1>>> opt;

	double obs_l = constraints[0];
	double obs_d = constraints[1];//当障碍物突出方向和目标点同向时为正，即其符号取决于目标点的位置
	double obs_v = constraints[2]; //障碍物绝对车速
	//cout << bound_condition[0] << " " << bound_condition[1] << " " << bound_condition[2] << endl;
	//cout << "l:" << l << " w:" << w << endl;

	int N = (int)(L_theta.size() / 2);

	double v = sqrt(ALATER / KMAX);

	for (auto a = L_theta.begin(); a != L_theta.begin() + N; a += 2)
	{
		alfa_tmp = *a;
		l1_tmp = Vehicle::_L_min(alfa_tmp, 2 * ERROR_KMAX);

		if (l1_tmp >= l_max)
			continue;

		x1_tmp = x0 + l1_tmp*cos(theta0);
		y1_tmp = y0 + l1_tmp*sin(theta0);

		// calculate l2&l3

		t = alfa_tmp - theta0;
		beita_tmp = theta3 + t;

		if (sin(beita_tmp) != 0)
		{
			l2_tmp = ((y3 - y1_tmp)*cos(theta3) - (x3 - x1_tmp)*sin(theta3)) / sin(beita_tmp);
			if (cos(theta3) != 0)
				l3_tmp = (x3 - x1_tmp + l2_tmp*cos(t)) / cos(theta3);
			else
				l3_tmp = (y3 - y1_tmp - l2_tmp*sin(t)) / sin(theta3);
		}
		else
		{
			l3_tmp = ((x3 - x1_tmp)*sin(t) + (y3 - y1_tmp)*cos(t)) / (2 * sin(beita_tmp));
			if (cos(t) != 0)
				l2_tmp = (x1_tmp - x3 + l3_tmp*cos(theta3)) / cos(t);
			else
				l2_tmp = (y3 - y1_tmp - l3_tmp*sin(theta3)) / sin(t);
		}

		if (l3_tmp < 0 || l2_tmp < 0)
			continue;

		//cout << std::min(l2_tmp, l1_tmp) << " " << (std::min(l2_tmp, l3_tmp)) << endl;
		//cout << (std::min(l2_tmp, l1_tmp) < *(a + 1)) << endl;
		//cout << (std::min(l2_tmp, l3_tmp) < _L_min(alfa_tmp + thetag)) << endl;
		//cout << *(a + 1) << " " << _L_min(alfa_tmp + thetag) << endl;

		if (beita_tmp > PI)
			beita_tmp = 2 * PI - beita_tmp;
		if (std::min(l2_tmp, l1_tmp) < *(a + 1) || std::min(l2_tmp, l3_tmp) < Vehicle::_L_min(beita_tmp) || beita_tmp < ALFA_MIN)
			continue;

		//cout << "l2 meets demand" << endl;
		//cout << "l3 meets demand" << endl;

		x2_tmp = x3 - l3_tmp*cos(theta3);
		y2_tmp = y3 - l3_tmp*sin(theta3);

		double t1 = l1_tmp / v;
		double t2 = (l1_tmp + l2_tmp) / v;

		if (sin(theta0) != 0)
		{
			double ld1 = (obs_d - 0.5*W*abs(cos(theta0)) - ERROR_OBS) / sin(theta0) - L_F_BA;
			double ld2 = (obs_d + 0.5*W*abs(cos(theta0)) - ERROR_OBS) / sin(theta0) + L - L_F_BA;
			double td1 = ld1 / v;
			double td2 = ld2 / v;
			if (l1_tmp < ld1)
				c = 1;
			else
			{
				double G4_1 = obs_d - 0.5*W / abs(cos(theta0)) - abs(tan(theta0))*(obs_l + obs_v*td1);
				G4_1 *= abs(cos(theta0));
				double G4_2;
				if (l1_tmp < ld2)
				{
					G4_2 = obs_d - 0.5*W / abs(cos(theta0)) - abs(tan(theta0))*(obs_l + obs_v*t1);
					G4_2 *= abs(cos(theta0));
				}
				else
				{
					G4_2 = obs_d - 0.5*W / abs(cos(theta0)) - abs(tan(theta0))*(obs_l + obs_v*td2);
					G4_2 *= abs(cos(theta0));
					double G4_3 = (l1_tmp - (L - L_F_BA))*abs(cos(theta0)) - 0.5*W*abs(sin(theta0)) - (obs_l + obs_v*t1);
					G4_2 = std::min(G4_2, G4_3);
				}
				c = std::min(G4_1, G4_2) - ERROR_OBS;
			}
		}
		else
			c = 1;
		if (c <= 0)
			continue;

		double beita_1 = atan2(y2_tmp - y1_tmp, x2_tmp - x1_tmp);
		if (sin(beita_1) != 0)
		{
			double ld4 = (obs_d - 0.5*W*abs(cos(beita_1)) - l1_tmp*sin(theta0) - ERROR_OBS) / sin(beita_1) - L_F_BA;
			double ld5 = (obs_d - 0.5*W*abs(cos(beita_1)) - l1_tmp*sin(theta0) - ERROR_OBS) / sin(beita_1) + L - L_F_BA;
			double td4 = (l1_tmp + ld4) / v;
			double td5 = (l1_tmp + ld5) / v;
			if (l2_tmp < ld4)
				c = 1;
			else
			{
				double G5_1 = obs_d - (y1_tmp - y0) + (x1_tmp - x0)*abs(tan(beita_1)) - 0.5*W / abs(cos(beita_1)) - (obs_l + obs_v*td4)*abs(tan(beita_1));
				G5_1 *= abs(cos(beita_1));
				double G5_2;
				if (l2_tmp < ld5)
				{
					G5_2 = obs_d - (y1_tmp - y0) + (x1_tmp - x0)*abs(tan(beita_1)) - 0.5*W / abs(cos(beita_1)) - (obs_l + obs_v*t2)*abs(tan(beita_1));
					G5_2 *= abs(cos(beita_1));
				}
				else
				{
					G5_2 = obs_d - (y1_tmp - y0) + (x1_tmp - x0)*abs(tan(beita_1)) - 0.5*W / abs(cos(beita_1)) - (obs_l + obs_v*td5)*abs(tan(beita_1));
					G5_2 *= abs(cos(beita_1));
					double G5_3 = (x2_tmp - x0) - (L - L_F_BA)*abs(cos(beita_1)) - 0.5*W*abs(sin(beita_1)) - (obs_l + obs_v*t2);
					G5_2 = std::min(G5_2, G5_3);
				}
				c = std::min(G5_1, G5_2) - ERROR_OBS;
			}
		}
		else
		{
			if (y2_tmp + 0.5*W*abs(cos(beita_1))>obs_d)
				c = x1_tmp - L + L_F_BA - (obs_l + obs_v*t1);
			else
				c = 1;
		}
		if (c <= 0)
			continue;

		if (std::min(y2_tmp, y3) + 0.5*W*abs(cos(theta3)) + ERROR_OBS > obs_d)
			c = (x3 - x0) - (L - L_F_BA)*abs(cos(theta3)) - 0.5*W*abs(sin(theta3)) - (obs_l + obs_v*(l1_tmp + l2_tmp + l3_tmp) / v) - ERROR_OBS;
		else
			c = 1;
		if (c <= 0)
			continue;

		result = true;
		Vector4d opt_data;
		opt_data << std::min({ l1_tmp + l2_tmp + l3_tmp }), l1_tmp, l2_tmp, l3_tmp;
		opt.push_back(opt_data);
	}
	auto min_i = opt.begin();
	double min = 1000;
	if (result == true)
	{
		//cout << "l1:" << l1 << " l2:" << l2 << " l3:" << l3 << " alfa:" << alfa << endl;
		auto i = opt.begin();
		for (; i != opt.end(); i++)
		{
			if (min > (*i)[0])
			{
				min_i = i;
				min = (*i)[0];
			}
		}

		x1 = start_node.x + (*min_i)[1] * cos(start_node.theta);      y1 = start_node.y + (*min_i)[1] * sin(start_node.theta);
		x2 = end_node.x - (*min_i)[3] * cos(end_node.theta);		  y2 = end_node.y - (*min_i)[3] * sin(end_node.theta);
		control->push_back(start_node.x); control->push_back(start_node.y);
		control->push_back(x1); control->push_back(y1);
		control->push_back(x2); control->push_back(y2);
		control->push_back(end_node.x); control->push_back(end_node.y);
	}

	return result;
}

bool Trajectory::turn(const Vehicle::Node &start_node, const Vehicle::Node &end_node, const double &angle, vector<double> *control)
{
	control->clear();

	Vehicle::Node local_start_node, local_end_node;
	orient_trans(start_node, end_node, angle, &local_start_node, &local_end_node);

	double xi = local_start_node.x;
	double yi = local_start_node.y;
	double theta_i = local_start_node.theta;
	double xg = local_end_node.x;
	double yg = local_end_node.y;
	double theta_g = local_end_node.theta;

	/*double xi = start_node.x;
	double yi = start_node.y;
	double angle_i = angle; //初始时刻车道中心线和车辆航向角的夹角，从中心线逆时针旋转为正，终止时刻车道中心线和车辆航向角的夹角为0
	double theta_i = norm_theta_0_2pi(-angle_i);
	double xg = (end_node.x - xi)*cos(start_node.theta + angle_i) + (end_node.y - yi)*sin(start_node.theta + angle_i) + xi;
	double yg = -(end_node.x - xi)*sin(start_node.theta + angle_i) + (end_node.y - yi)*cos(start_node.theta + angle_i) + yi;
	double theta_g = norm_theta_0_2pi(end_node.theta - start_node.theta - angle_i);

	double angle_i = angle; //初始时刻车道中心线和车辆航向角的夹角，从中心线逆时针旋转为正, 终止时刻车道中心线和车辆航向角的夹角默认为0

	int sign_y;
	int sign_x;
	if ((xg - xi) < 0)
	{
		sign_x = -1;
		if ((yg - yi) > 0)
			sign_y = -1;
		else
			sign_y = 1;
	}
	else
	{
		sign_x = 1;
		if ((yg - yi) < 0)
			sign_y = -1;
		else
			sign_y = 1;
	}*/

	double x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
	double l1, l2, l3, l4, alfa_tmp, beita_tmp, gama_tmp, l1_tmp, l2_tmp, l3_tmp, l4_tmp;
	double delta_1, delta_2, delta_3;
	vector<Vector6d> *subopt = new vector<Vector6d>;
	vector<Vector6d> *opt = new vector<Vector6d>;
	bool result = false;
	bool suboptimal = false;
	double t1, t2;

	double step = 0.5;
	double exceed_allow_x = 0;
	double exceed_allow_y = 2; //超出部分到车道中心线的距离
	for (auto a = L_theta.begin(); a != L_theta.end() - 4; a += 2)
	{
		if (result == true)
			break;
		// calculate l1
		alfa_tmp = *a;
		l1_tmp = *(a + 1) - step;

		while (result == false)
		{
			l1_tmp += step;

			// calculate l2&l3
			x1_tmp = xi + l1_tmp;
			y1_tmp = yi;
			if ((x1_tmp - xg) > exceed_allow_x)
				break;
			/*if (-sign_y*l1_tmp*sin(angle_i) > exceed_allow_y)
				break;*/

			for (auto b = L_theta.begin(); b != L_theta.end(); b += 2)
			{
				if (result == true)
					break;
				beita_tmp = *b;
				l4_tmp = *(b + 1) - step;

				while (result == false)
				{
					l4_tmp += step;

					x3_tmp = xg - l4_tmp*cos(theta_g);
					y3_tmp = yg - l4_tmp*sin(theta_g);
					if ((y3_tmp - yi) < exceed_allow_x)
						break;

					t1 = alfa_tmp;
					t2 = theta_g + beita_tmp;
					if (sin(t2) == 0 && sin(t1) != 0)
					{
						l2_tmp = (y3_tmp - y1_tmp) / sin(t1);
						l3_tmp = (x1_tmp - x3_tmp - l2_tmp*cos(t1)) / cos(t2);
					}
					else if (sin(t2) != 0 && sin(t1 + t2) != 0)
					{
						l2_tmp = ((x1_tmp - x3_tmp)*sin(t2) + (y3_tmp - y1_tmp)*cos(t2)) / sin(t1 + t2);
						l3_tmp = (y1_tmp - y3_tmp + l2_tmp*sin(t1)) / sin(t2);
					}
					else
						continue;

					if (l2_tmp > 0 && l3_tmp > 0)
					{
						if (std::min(l1_tmp, l2_tmp) - ERROR_KMAX > *(a + 1) && std::min(l4_tmp, l3_tmp) - ERROR_KMAX > *(b + 1))
						{
							gama_tmp = acos((pow(l2_tmp, 2) + pow(l3_tmp, 2) - pow(dist(x1_tmp, y1_tmp, x3_tmp, y3_tmp), 2)) / (2 * l2_tmp*l3_tmp));

							if (gama_tmp > PI / 2 && std::min(l2_tmp, l3_tmp) - ERROR_KMAX > Vehicle::_L_min(gama_tmp))
							{
								x2_tmp = x1_tmp - l2_tmp*cos(t1);
								y2_tmp = y1_tmp + l2_tmp*sin(t1);

								delta_1 = std::min(l1_tmp, l2_tmp) - *(a + 1);
								delta_2 = std::min(l2_tmp, l3_tmp) - Vehicle::_L_min(gama_tmp);
								delta_3 = std::min(l3_tmp, l4_tmp) - *(b + 1);
								vector<double> delta_set = { delta_1, delta_2, delta_3 };
								double delta = *std::min_element(delta_set.begin(), delta_set.end());
								delta = l1_tmp + l2_tmp + l3_tmp + l4_tmp;

								if (-l1_tmp*sin(theta_i) < exceed_allow_y && (l1_tmp*sin(theta_i) - l2_tmp*sin(t1 + theta_i)) < 2 * exceed_allow_y&&-l3_tmp*sin(beita_tmp) < exceed_allow_y)
								{
									result = true;
									Vector6d optdata;
									optdata << delta, l1_tmp, l2_tmp, l3_tmp, l4_tmp, alfa_tmp;
									opt->emplace_back(optdata);
								} //第二个条件表示y2不能超出yi多少范围
								else
								{
									suboptimal = true;
									Vector6d suboptdata;
									suboptdata << delta, l1_tmp, l2_tmp, l3_tmp, l4_tmp, alfa_tmp;
									subopt->emplace_back(suboptdata);
								}
							}
						}
					}
				}
			}
		}
	}

	auto min_i = opt->begin();
	double min = 0;
	if (result == true)
	{
		auto i = opt->begin();
		for (; i != opt->end(); i++)
		{
			if (min < (*i)[0])
			{
				min_i = i;
				min = (*i)[0];
			}
		}
	}
	else
	{
		if (suboptimal == true)
		{
			auto i = subopt->begin();
			for (; i != subopt->end(); i++)
			{
				if (min < (*i)[0])
				{
					min_i = i;
					min = (*i)[0];
				}
			}
			result = true;
		}
		else
			return false;
	}

	l1 = (*min_i)[1];
	l2 = (*min_i)[2];
	l3 = (*min_i)[3];
	l4 = (*min_i)[4];
	double alfa = (*min_i)[5];
	double x1 = start_node.x + l1*cos(start_node.theta);      double y1 = start_node.y + l1*sin(start_node.theta);
	double x2 = x1 - l2*cos(alfa);						  double y2 = y1 + l2*sin(alfa);
	double x3 = end_node.x - l3*cos(end_node.theta);		  double y3 = end_node.y - l3*sin(end_node.theta);

	control->push_back(start_node.x); control->push_back(start_node.y);
	control->push_back(x1); control->push_back(y1);
	control->push_back(x2); control->push_back(y2);
	control->push_back(x3); control->push_back(y3);
	control->push_back(end_node.x); control->push_back(end_node.y);
	delete subopt, opt;
	return result;
}

bool Trajectory::turn_c(const Vehicle::Node &start_node, const Vehicle::Node &end_node, const double &angle, const vector<double> &constraints, vector<double> *control)
{
	control->clear();

	Vehicle::Node local_start_node, local_end_node;
	orient_trans(start_node, end_node, angle, &local_start_node, &local_end_node);

	double xi = local_start_node.x;
	double yi = local_start_node.y;
	double theta_i = local_start_node.theta;
	double xg = local_end_node.x;
	double yg = local_end_node.y;
	double theta_g = local_end_node.theta;

	/*double xi = start_node.x;
	double yi = start_node.y;
	double angle_i = angle; //初始时刻车道中心线和车辆航向角的夹角，从中心线逆时针旋转为正，终止时刻车道中心线和车辆航向角的夹角为0
	double theta_i = norm_theta_0_2pi( - angle_i);
	double xg = (end_node.x - xi)*cos(start_node.theta + angle_i) + (end_node.y - yi)*sin(start_node.theta + angle_i) + xi;
	double yg = -(end_node.x - xi)*sin(start_node.theta + angle_i) + (end_node.y - yi)*cos(start_node.theta + angle_i) + yi;
	double theta_g = norm_theta_0_2pi(end_node.theta - start_node.theta - angle_i);

	int sign_y;
	int sign_x;
	if ((xg - xi) < 0)
	{
		sign_x = -1;
		if ((yg - yi) > 0)
			sign_y = -1;
		else
			sign_y = 1;
	}
	else
	{
		sign_x = 1;
		if ((yg - yi) < 0)
			sign_y = -1;
		else
			sign_y = 1;
	}*/

	double x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
	double l1, l2, l3, l4, alfa_tmp, beita_tmp, gama_tmp, l1_tmp, l2_tmp, l3_tmp, l4_tmp;
	double delta_1, delta_2, delta_3;
	vector<Vector7d> *subopt = new vector<Vector7d>;
	vector<Vector7d> *opt = new vector<Vector7d>;
	bool result = false;
	bool suboptimal = false;

	double l_f = constraints[0]; //从后轴中心算起到约束头
	double d_f = constraints[1]; //从后轴中心的车道线的横向距离 与车道线垂直
	double l_r = constraints[2];
	double d_r = constraints[3];
	/*if (thetag == PI / 4)
	return result;*/

	double step = 0.5;
	double exceed_allow_x = 0;
	double exceed_allow_y = 2; //超出部分到车道中心线的距离
	double t1, t2, t3;//特征角，t1为B到C与x轴夹角，t2为D到C与x轴夹角

	for (auto a = L_theta.begin(); a != L_theta.end() - 4; a += 2)
	{
		if (result == true/* || is_exceed == true*/)
			break;
		// calculate l1
		alfa_tmp = *a;
		t1 = alfa_tmp - theta_i;
		if (l_f != 0 && d_f != 0)
			l1_tmp = (d_f + l_f*tan(t1)) / (cos(theta_i)*tan(t1) - sin(theta_i));
		else
			l1_tmp = *(a + 1) - step;

		while (result == false)
		{
			l1_tmp += step;

			// calculate l2&l3
			x1_tmp = xi + l1_tmp*cos(theta_i);
			y1_tmp = yi + l1_tmp*sin(theta_i);
			if ((x1_tmp - xg) > exceed_allow_x)
				break;
			/*if (-sign_y*l1_tmp*sin(angle_i) > exceed_allow_y)
			break;*/

			for (auto b = L_theta.begin(); b != L_theta.end(); b += 2)
			{
				if (result == true)
					break;
				beita_tmp = *b;
				t2 = theta_g + beita_tmp;
				if (d_r != 0 && l_r != 0)
					l4_tmp = l_r + d_r / tan(beita_tmp);
				else
					l4_tmp = *(b + 1) - step;

				while (result == false)
				{
					l4_tmp += step;

					x3_tmp = xg - l4_tmp*cos(theta_g);
					y3_tmp = yg - l4_tmp*sin(theta_g);
					if ((y3_tmp - yi) < exceed_allow_x)
						break;
					/*if (sign_y*l4_tmp*sin(angle_g) > exceed_allow_y)
					break;*/

					if (sin(t2) == 0 && sin(t1) != 0)
					{
						l2_tmp = (y3_tmp - y1_tmp) / sin(t1);
						l3_tmp = (x1_tmp - x3_tmp - l2_tmp*cos(t1)) / cos(t2);
					}
					else if (sin(t2) != 0 && sin(t1 + t2) != 0)
					{
						l2_tmp = ((x1_tmp - x3_tmp)*sin(t2) + (y3_tmp - y1_tmp)*cos(t2)) / sin(t1 + t2);
						l3_tmp = (y1_tmp - y3_tmp + l2_tmp*sin(t1)) / sin(t2);
					}
					else
						continue;

					if (l2_tmp > 0 && l3_tmp > 0)
					{
						if (std::min(l1_tmp, l2_tmp) - ERROR_KMAX > *(a + 1) && std::min(l4_tmp, l3_tmp) - ERROR_KMAX > *(b + 1))
						{
							gama_tmp = acos((pow(l2_tmp, 2) + pow(l3_tmp, 2) - pow(dist(x1_tmp, y1_tmp, x3_tmp, y3_tmp), 2)) / (2 * l2_tmp*l3_tmp));

							if (gama_tmp > 5*PI / 9 && std::min(l2_tmp, l3_tmp) - ERROR_KMAX > Vehicle::_L_min(gama_tmp))
							{
								x2_tmp = x1_tmp - l2_tmp*cos(t1);
								y2_tmp = y1_tmp + l2_tmp*sin(t1);

								suboptimal = true;

								if (l_f != 0 && d_f != 0)
								{
									/*if ((L_F_BA + l1_tmp)*sin(theta_i) + 0.5*W*sign_y*cos(theta_i) < d_f)
										if ((L_F_BA + l1_tmp)*std::abs(cos(theta_i)) + 0.5*W*std::abs(sin(theta_i)) < l_f + ERROR_OBS)
										{
											cout << "起点折角不满足约束" << endl;
											suboptimal = false;
											continue;
										}*/
									
									if (d_f > (y1_tmp - yi))
									{
										if (l_f*cos(theta_i) > l1_tmp - d_f*sin(theta_i))
										{
											if (std::min(d_f*std::abs(cos(theta_i)), d_f*std::abs(cos(theta_i)) - (l1_tmp - d_f*sin(theta_i))* std::abs(tan(theta_i))) < 0.5*W + ERROR_OBS)
											{
												suboptimal = false;
												//cout << "d_f位于x1外，与l1相交，不满足约束条件" << endl;
												continue;
											}
										}
										else
										{
											if (std::min(d_f*std::abs(cos(theta_i)), d_f*std::abs(cos(theta_i)) - l_f*sin(theta_i)) < 0.5*W + ERROR_OBS)
											{
												suboptimal = false;
												//cout << "d_f位于x1 nei，与l1相交，不满足约束条件" << endl;
												continue;
											}
										}
										if (((l_f - std::abs(x1_tmp - xi))* std::abs(sin(t1)) + 0.5*W) >= ((d_f - std::abs(y1_tmp - yi))*std::abs(cos(t1)) - ERROR_OBS))
										{
											suboptimal = false;
											//cout << "d_f位于y1外，不满足约束条件" << endl;
											continue;
										}
										else
										{
											if (d_f > (y2_tmp - yi))
											{
												if (((l_f - std::abs(x2_tmp - xi))* std::abs(sin(t2)) + 0.5*W) >= ((d_f - std::abs(y2_tmp - yi))*std::abs(cos(t2)) - ERROR_OBS))
												{
													suboptimal = false;
												//	cout << "d_f位于y2外，不满足约束条件" << endl;
													continue;
												}
												else
												{
													if (d_f > (y3_tmp - yi))
													{
														if (((l_f - std::abs(x3_tmp - xi))* std::abs(sin(theta_g)) + 0.5*W) >= ((d_f - std::abs(y3_tmp - yi))*std::abs(cos(theta_g)) - ERROR_OBS))
														{
															suboptimal = false;
												//			cout << "d_f位于y3外，不满足约束条件" << endl;
															continue;
														}
													}
												}
											}
										}
									}
									else
									{
										if ((l_f* std::abs(sin(theta_i)) + 0.5*W) >= (d_f*std::abs(cos(theta_i)) - ERROR_OBS))
										{
											suboptimal = false;
											//cout << "d_f位于l1处，不满足约束条件" << endl;
											continue;
										}
									}
								}

								//if (std::abs(l1_tmp - 1.51841) < 10e-3&&std::abs(l2_tmp - 22.1177) < 10e-3&&std::abs(l3_tmp - 9.78963) < 10e-3&&std::abs(l4_tmp - 2.01841) < 10e-3)
									//cout << "attention" << endl;
								//判断起点处的约束
								//cout << endl;
								if (l_r != 0 && d_r != 0)
								{
									t3 = beita_tmp + gama_tmp;
									/*	if (-L_F_BA*cos(beita_tmp) - 0.5*W*std::abs(sin(beita_tmp)) + l_r > l4_tmp)
											if (-L_F_BA*sin(beita_tmp) - 0.5*cos(beita_tmp) + ERROR_OBS > d_r)
											{
											suboptimal = false;
											continue;
											}*/
										if ((l4_tmp + d_r / std::abs(tan(beita_tmp)) - l_r)*std::abs(sin(beita_tmp)) < 0.5*W + ERROR_OBS)
										{
											suboptimal = false;
											//cout << "d_r位于l3处，不满足约束条件" << endl;
											continue;
										}
										if (d_r > l3_tmp*sin(beita_tmp))
										{
											if ((l4_tmp - l3_tmp*cos(beita_tmp) - (d_r - l3_tmp*sin(beita_tmp)) / tan(t3) - l_r)*std::abs(sin(t3)) < 0.5*W + ERROR_OBS)
											{
												suboptimal = false;
												//cout << "d_r位于l2处，不满足约束条件" << endl;
												continue;
											}
											if (d_r > l3_tmp*sin(beita_tmp) + l2_tmp*std::abs(sin(t3)))
											{
												if ((l4_tmp - l3_tmp*cos(beita_tmp) + l2_tmp*std::abs(cos(t3)) + (d_r - l3_tmp*sin(beita_tmp) - l2_tmp*std::abs(sin(t3))) / tan(theta_g - theta_i) - l_r)*std::abs(sin(theta_g - theta_i)) < 0.5*W + ERROR_OBS)
												{
													suboptimal = false;
												//	cout << "d_r位于l1处，不满足约束条件" << endl;
													continue;
												}
											} //终止位置为(d_r < l3_tmp*sin(beita_tmp) + l2_tmp*std::abs(sin(t3)) + l1_tmp*std::abs(sin(theta_g - theta_i)))
										}
									//}
								}

								delta_1 = std::min(l1_tmp, l2_tmp) - *(a + 1);
								delta_2 = std::min(l2_tmp, l3_tmp) - Vehicle::_L_min(gama_tmp);
								delta_3 = std::min(l3_tmp, l4_tmp) - *(b + 1);

								if (suboptimal == true)
								{					
									if (-l1_tmp*sin(theta_i) < exceed_allow_y && (y2_tmp - yi) < exceed_allow_y&&-l3_tmp*sin(beita_tmp) < exceed_allow_y)
									{
										result = true;
										Vector7d optdata;
										optdata << std::min({ delta_1, delta_2, delta_3 }), x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
										opt->emplace_back(optdata);
									}
									else
									{
										suboptimal = true;
										Vector7d suboptdata;
										suboptdata << std::min({ delta_1, delta_2, delta_3 }), x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
										subopt->emplace_back(suboptdata);
									}
								}
							}
						}
					}
					//else
					//	break;
				}
			}
		}
	}

	auto min_i = opt->begin();
	double min = 1000;
	if (result == true)
	{
		//cout << "l1:" << l1 << " l2:" << l2 << " l3:" << l3 << " alfa:" << alfa << endl;
		auto i = opt->begin();
		for (; i != opt->end(); i++)
		{
			if (min > (*i)[0])
			{
				min_i = i;
				min = (*i)[0];
			}
		}
	}
	else
	{
		if (suboptimal == true)
		{
			auto i = subopt->begin();
			for (; i != subopt->end(); i++)
			{
				if (min > (*i)[0])
				{
					min_i = i;
					min = (*i)[0];
				}
			}
			result = true;
		}
		else
			return false;
	}

	l2 = dist((*min_i)[1], (*min_i)[2], (*min_i)[3], (*min_i)[4]);
	l3 = dist((*min_i)[5], (*min_i)[6], (*min_i)[3], (*min_i)[4]);
	l4 = dist((*min_i)[5], (*min_i)[6], xg, yg);
	l1 = dist((*min_i)[1], (*min_i)[2], xi, yi);
	alfa_tmp = PI - atan2((*min_i)[4] - (*min_i)[2], (*min_i)[3] - (*min_i)[1]);
	beita_tmp = PI - atan2((*min_i)[5] - (*min_i)[3], (*min_i)[6] - (*min_i)[4]);
	cout << "l1:" << l1_tmp << " l2:" << l2_tmp << " l3:" << l3_tmp << " l4:" << l4_tmp << " alfa:" << alfa_tmp << " alfa:" << beita_tmp << endl;

	control->push_back(start_node.x); control->push_back(start_node.y);
	for (int i = 1; i <= 6; i += 2)
	{
		control->push_back(((*min_i)[i] - xi) * cos(start_node.theta + angle) - ((*min_i)[i + 1] - yi) * sin(start_node.theta + angle) + xi);
		control->push_back(((*min_i)[i] - xi) * sin(start_node.theta + angle) + ((*min_i)[i + 1] - yi) * cos(start_node.theta + angle) + yi);
		cout << (*min_i)[i] << " " << (*min_i)[i + 1] << endl;
	}
	control->push_back(end_node.x); control->push_back(end_node.y);
	delete subopt, opt;
	return result;
}

bool Trajectory::U_turn(const Vehicle::Node &start_node, const Vehicle::Node &end_node, const double &angle, vector<double> *control)
{
	control->clear();

	double xi = start_node.x;
	double yi = start_node.y;
	double angle_i = angle; //初始时刻车道中心线和车辆航向角的夹角，从中心线逆时针旋转为正, 终止时刻车道中心线和车辆航向角的夹角默认为0
	double xg = (end_node.x - xi)*cos(start_node.theta) + (end_node.y - yi)*sin(start_node.theta) + xi;
	double yg = -(end_node.x - xi)*sin(start_node.theta) + (end_node.y - yi)*cos(start_node.theta) + yi;
	//double angle_g = angle[1]; 
	double theta_g = norm_theta_0_2pi(end_node.theta - start_node.theta);

	int sign_y;
	//int sign_x;
	//if ((xg - xi) < 0)
	//{
	//	sign_x = -1;
	//	if ((yg - yi) > 0)
	//		sign_y = -1;
	//	else
	//		sign_y = 1;
	//}
	//else
	//{
	//	sign_x = 1;
		if ((yg - yi) < 0)
			sign_y = -1;
		else
			sign_y = 1;
	//}

	double x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
	double l1, l2, l3, l4, alfa_tmp, beita_tmp, gama_tmp, l1_tmp, l2_tmp, l3_tmp, l4_tmp;
	double delta_1, delta_2, delta_3;
	vector<Vector7d> *subopt = new vector<Vector7d>;
	vector<Vector7d> *opt = new vector<Vector7d>;
	bool result = false;
	bool suboptimal = false;
	double t1, t2;

	double step = 0.5;
	double exceed_allow_x = 0;
	double exceed_allow_crossing = 4;
	double exceed_allow_y = 2; //超出部分到车道中心线的距离
	for (auto a = L_theta.begin(); a != L_theta.end() - 4; a += 2)
	{
		if (result == true)
			break;
		// calculate l1
		alfa_tmp = *a;
		l1_tmp = *(a + 1) - step;

		while (result == false)
		{
			l1_tmp += step;

			// calculate l2&l3
			x1_tmp = xi + l1_tmp;
			y1_tmp = yi;
			if ((x1_tmp - xg) > exceed_allow_x)
				break;
			/*if (-sign_y*l1_tmp*sin(angle_i) > exceed_allow_y)
			break;*/

			for (auto b = L_theta.begin(); b != L_theta.end(); b += 2)
			{
				if (result == true)
					break;
				beita_tmp = *b;
				l4_tmp = *(b + 1) - step;

				while (result == false)
				{
					l4_tmp += step;

					x3_tmp = xg - l4_tmp*cos(theta_g);
					y3_tmp = yg - l4_tmp*sin(theta_g);
					if (sign_y*(y3_tmp - yi) < exceed_allow_x)
						break;
					if (std::abs((xg - x3_tmp) / cos(angle)) > exceed_allow_crossing)
						break;

					t1 = alfa_tmp;
					t2 = theta_g + sign_y*beita_tmp;
					if (sin(t2) == 0 && sin(t1) != 0)
					{
						l2_tmp = (y3_tmp - y1_tmp) / sin(t1);
						l3_tmp = (x1_tmp - x3_tmp - l2_tmp*cos(t1)) / cos(t2);
					}
					else if (sin(t2) != 0 && sin(t1 + t2) != 0)
					{
						l2_tmp = ((x1_tmp - x3_tmp)*sin(t2) + (y3_tmp - y1_tmp)*cos(t2)) / sin(t1 + t2);
						l3_tmp = (y1_tmp - y3_tmp + l2_tmp*sin(t1)) / sin(t2);
					}
					else
						continue;

					if (l2_tmp > 0 && l3_tmp > 0)
					{
						if (std::min(l1_tmp, l2_tmp) - ERROR_KMAX > *(a + 1) && std::min(l4_tmp, l3_tmp) - ERROR_KMAX > *(b + 1))
						{
							gama_tmp = acos((pow(l2_tmp, 2) + pow(l3_tmp, 2) - pow(dist(x1_tmp, y1_tmp, x3_tmp, y3_tmp), 2)) / (2 * l2_tmp*l3_tmp));

							if (gama_tmp > PI / 2 && std::min(l2_tmp, l3_tmp) - ERROR_KMAX > Vehicle::_L_min(gama_tmp))
							{
								x2_tmp = x1_tmp - l2_tmp*cos(t1);
								y2_tmp = y1_tmp + l2_tmp*sin(t1);

								delta_1 = std::min(l1_tmp, l2_tmp) - *(a + 1);
								delta_2 = std::min(l2_tmp, l3_tmp) - Vehicle::_L_min(gama_tmp);
								delta_3 = std::min(l3_tmp, l4_tmp) - *(b + 1);

								if (-sign_y*l1_tmp*sin(angle_i) < exceed_allow_y && (l1_tmp*sin(angle_i) - l2_tmp*sin(t1 + angle_i)) < 2 * exceed_allow_y&&-l3_tmp*sin(beita_tmp) < exceed_allow_y)
								{
									result = true;
									Vector7d optdata;
									optdata << std::min({ delta_1, delta_2, delta_3 }), x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
									opt->emplace_back(optdata);
								}
								else
								{
									suboptimal = true;
									Vector7d suboptdata;
									suboptdata << std::min({ delta_1, delta_2, delta_3 }), x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
									subopt->emplace_back(suboptdata);
								}
							}
						}
					}
				}
			}
		}
	}

	auto min_i = opt->begin();
	double min = 0;
	if (result == true)
	{
		auto i = opt->begin();
		for (; i != opt->end(); i++)
		{
			if (min < (*i)[0])
			{
				min_i = i;
				min = (*i)[0];
			}
		}
	}
	else
	{
		if (suboptimal == true)
		{
			auto i = subopt->begin();
			for (; i != subopt->end(); i++)
			{
				if (min < (*i)[0])
				{
					min_i = i;
					min = (*i)[0];
				}
			}
			result = true;
		}
		else
			return false;
	}

	if (result == true)
	{
		control->push_back(start_node.x); control->push_back(start_node.y);
		for (int i = 1; i <= 6; i += 2)
		{
			control->push_back(((*min_i)[i] - xi) * cos(start_node.theta) - ((*min_i)[i + 1] - yi) * sin(start_node.theta) + xi);
			control->push_back(((*min_i)[i] - xi) * sin(start_node.theta) + ((*min_i)[i + 1] - yi) * cos(start_node.theta) + yi);
			cout << (*min_i)[i] << " " << (*min_i)[i + 1] << endl;
		}
		control->push_back(end_node.x); control->push_back(end_node.y);
	}
	delete subopt, opt;
	return result;
}

bool Trajectory::U_turn_c(const Vehicle::Node &start_node, const Vehicle::Node &end_node, const double &angle, const vector<double> &constraints, vector<double> *control)
{
	control->clear();

	double xi = start_node.x;
	double yi = start_node.y;
	double angle_i = angle; //初始时刻车道中心线和车辆航向角的夹角，从中心线逆时针旋转为正，终止时刻车道中心线和车辆航向角的夹角为0
	double theta_i = norm_theta_0_2pi(-angle_i);
	double xg = (end_node.x - xi)*cos(start_node.theta + angle_i) + (end_node.y - yi)*sin(start_node.theta + angle_i) + xi;
	double yg = -(end_node.x - xi)*sin(start_node.theta + angle_i) + (end_node.y - yi)*cos(start_node.theta + angle_i) + yi;
	double theta_g = norm_theta_0_2pi(end_node.theta - start_node.theta - angle_i);

	int sign_y;
	int sign_x;
	if ((xg - xi) < 0)
	{
		sign_x = -1;
		if ((yg - yi) > 0)
			sign_y = -1;
		else
			sign_y = 1;
	}
	else
	{
		sign_x = 1;
		if ((yg - yi) < 0)
			sign_y = -1;
		else
			sign_y = 1;
	}

	double x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
	double l1, l2, l3, l4, alfa_tmp, beita_tmp, gama_tmp, l1_tmp, l2_tmp, l3_tmp, l4_tmp;
	double delta_1, delta_2, delta_3;
	vector<Vector7d> *subopt = new vector<Vector7d>;
	vector<Vector7d> *opt = new vector<Vector7d>;
	bool result = false;
	bool suboptimal = false;

	double l = constraints[0]; //从后轴中心算起到约束头
	double d = constraints[1]; //从后轴中心的车道线的横向距离 与车道线垂直

	double step = 0.5;
	double exceed_allow_x = 0;
	double exceed_allow_crossing = 5;
	double exceed_allow_y = 2; //超出部分到车道中心线的距离
	double t1, t2, t3;//特征角，t1为B到C与x轴夹角，t2为D到C与x轴夹角
	for (auto a = L_theta.begin(); a != L_theta.end() - 4; a += 2)
	{
		if (result == true/* || is_exceed == true*/)
			break;
		// calculate l1
		alfa_tmp = *a;
		l1_tmp = *(a + 1) - step;

		while (result == false)
		{
			l1_tmp += step;

			// calculate l2&l3
			x1_tmp = xi + l1_tmp*cos(theta_i);
			y1_tmp = yi + l1_tmp*sin(theta_i);
			if (sign_x*(x1_tmp - xg) > exceed_allow_x)
				break;

			for (auto b = L_theta.begin(); b != L_theta.end(); b += 2)
			{
				if (result == true)
					break;
				beita_tmp = *b;
				l4_tmp = *(b + 1) - step;

				while (result == false)
				{
					l4_tmp += step;

					x3_tmp = xg - l4_tmp*cos(theta_g);
					y3_tmp = yg - l4_tmp*sin(theta_g);
					if (sign_x*sign_y*(y3_tmp - yi) < exceed_allow_x)
						break;
					if (std::abs((xg - x3_tmp) / cos(angle)) > l + exceed_allow_crossing)
						break;

					t1 = alfa_tmp - sign_y*theta_i;
					t2 = theta_g + sign_y*beita_tmp;
					if (sin(t2) == 0 && sin(t1) != 0)
					{
						l2_tmp = (y3_tmp - y1_tmp) / sin(t1);
						l3_tmp = (x1_tmp - x3_tmp - l2_tmp*cos(t1)) / cos(t2);
					}
					else if (sin(t2) != 0 && sin(t1 + t2) != 0)
					{
						l2_tmp = ((x1_tmp - x3_tmp)*sin(t2) + (y3_tmp - y1_tmp)*cos(t2)) / sin(t1 + t2);
						l3_tmp = (y1_tmp - y3_tmp + l2_tmp*sin(t1)) / sin(t2);
					}
					else
						continue;

					if (l2_tmp > 0 && l3_tmp > 0)
					{
						if (std::min(l1_tmp, l2_tmp) - ERROR_KMAX > *(a + 1) && std::min(l4_tmp, l3_tmp) - ERROR_KMAX > *(b + 1))
						{
							gama_tmp = acos((pow(l2_tmp, 2) + pow(l3_tmp, 2) - pow(dist(x1_tmp, y1_tmp, x3_tmp, y3_tmp), 2)) / (2 * l2_tmp*l3_tmp));

							if (gama_tmp > 5 * PI / 9 && std::min(l2_tmp, l3_tmp) - ERROR_KMAX > Vehicle::_L_min(gama_tmp))
							{
								x2_tmp = x1_tmp - l2_tmp*cos(t1);
								y2_tmp = y1_tmp + l2_tmp*sin(t1);

								suboptimal = true;

								if (l != 0 && d != 0)
								{									
									if (d > sign_x*sign_y*(y1_tmp - yi))
									{
										if (l*cos(theta_i) > l1_tmp - d*sin(theta_i))
										{
											if (std::min(d*std::abs(cos(theta_i)), d*std::abs(cos(theta_i)) - (l1_tmp - d*sin(theta_i))* std::abs(tan(theta_i))) < 0.5*W + ERROR_OBS)
											{
												suboptimal = false;
												cout << "d_f位于x1外，与l1相交，不满足约束条件" << endl;
												continue;
											}
										}
										else
										{
											if (std::min(d*std::abs(cos(theta_i)), d*std::abs(cos(theta_i)) - l*sin(theta_i)) < 0.5*W + ERROR_OBS)
											{
												suboptimal = false;
												cout << "d_f位于x1 nei，与l1相交，不满足约束条件" << endl;
												continue;
											}
										}
										if (((l - std::abs(x1_tmp - xi))* std::abs(sin(t1)) + 0.5*W) >= ((d - std::abs(y1_tmp - yi))*std::abs(cos(t1)) - ERROR_OBS))
										{
											suboptimal = false;
											cout << "d_f位于y1外，不满足约束条件" << endl;
											continue;
										}
										else
										{
											if (d > sign_x*sign_y*(y2_tmp - yi))
											{
												if (((l - std::abs(x2_tmp - xi))* std::abs(sin(t2)) + 0.5*W) >= ((d - std::abs(y2_tmp - yi))*std::abs(cos(t2)) - ERROR_OBS))
												{
													suboptimal = false;
													cout << "d_f位于y2外，不满足约束条件" << endl;
													continue;
												}
												else
												{
													if (d > sign_x*sign_y*(y3_tmp - yi))
													{
														if (((l - std::abs(x3_tmp - xi))* std::abs(sin(theta_g)) + 0.5*W) >= ((d - std::abs(y3_tmp - yi))*std::abs(cos(theta_g)) - ERROR_OBS))
														{
															suboptimal = false;
															cout << "d_f位于y3外，不满足约束条件" << endl;
															continue;
														}
													}
												}
											}
										}
									}
									else
									{
										if ((l* std::abs(sin(theta_i)) + 0.5*W) >= (d*std::abs(cos(theta_i)) - ERROR_OBS))
										{
											suboptimal = false;
											cout << "d_f位于l1处，不满足约束条件" << endl;
											continue;
										}
									}
								}

								delta_1 = std::min(l1_tmp, l2_tmp) - *(a + 1);
								delta_2 = std::min(l2_tmp, l3_tmp) - Vehicle::_L_min(gama_tmp);
								delta_3 = std::min(l3_tmp, l4_tmp) - *(b + 1);

								if (suboptimal == true)
								{
									if (-sign_y*l1_tmp*sin(angle_i) < exceed_allow_y && (y2_tmp - yi) < exceed_allow_y&&-l3_tmp*sin(beita_tmp) < exceed_allow_y)
									{
										result = true;
										Vector7d optdata;
										optdata << std::min({ delta_1, delta_2, delta_3 }), x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
										opt->emplace_back(optdata);
									}
									else
									{
										suboptimal = true;
										Vector7d suboptdata;
										suboptdata << std::min({ delta_1, delta_2, delta_3 }), x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
										subopt->emplace_back(suboptdata);
									}
								}
							}
						}
					}
				}
			}
		}
	}

	auto min_i = opt->begin();
	double min = 1000;
	if (result == true)
	{
		auto i = opt->begin();
		for (; i != opt->end(); i++)
		{
			if (min > (*i)[0])
			{
				min_i = i;
				min = (*i)[0];
			}
		}
	}
	else
	{
		if (suboptimal == true)
		{
			auto i = subopt->begin();
			for (; i != subopt->end(); i++)
			{
				if (min > (*i)[0])
				{
					min_i = i;
					min = (*i)[0];
				}
			}
			result = true;
		}
	}
	if (result == true)
	{
		control->push_back(start_node.x); control->push_back(start_node.y);
		control->push_back((*min_i)[1]); control->push_back((*min_i)[2]);
		control->push_back((*min_i)[3]); control->push_back((*min_i)[4]);
		control->push_back((*min_i)[5]); control->push_back((*min_i)[6]);
		control->push_back(end_node.x); control->push_back(end_node.y);
	}
	delete subopt, opt;
	return result;
}

int Trajectory::is_force_extend_safe(const Vehicle::Node &startnode, const Vehicle::Node &startnode_r, const double &step, const vector<double> &control, double *le, vector<Vehicle::Node> *path)
{
	vector<double> ctrl = control;
	orient_trans_vehicle_global(startnode, &ctrl);

	//下面生成的曲线是大地坐标，因此可以直接用于碰撞检测
	ts::BSpline bspline = ts::BSpline(3, 2, ctrl.size() - 1, TS_CLAMPED);
	vector<ts::rational> ctrl_points = bspline.ctrlp();
	ctrl_points[0] = ctrl[0];				ctrl_points[1] = ctrl[1];
	for (int i = 2; i < ctrl_points.size(); i += 4)
	{
		ctrl_points[i] = 0.5*(ctrl[1 + i / 2] + ctrl[i / 2 - 1]);
		ctrl_points[i + 1] = 0.5*(ctrl[2 + i / 2] + ctrl[i / 2]);
		ctrl_points[i + 2] = ctrl[1 + i / 2];
		ctrl_points[i + 3] = ctrl[2 + i / 2];
	}
	bspline.setCtrlp(ctrl_points);  //在原控制点中插入中点生成所需要的B样条曲线
	//std::ofstream outfile;
	//outfile.open("E:\\postgraduate\\codes\\PLAN\\plan20161130\\validate\\attachment.txt");
	int issafe = 2;
	double du = 0.01;
	double u = 0.;
	double s = 0.;
	int count = 0;
	*le = 0;
	double le_tmp = 0;
	int time = (int)std::floor(step / SAFESTEP);
	vector<double> old_tmp{ startnode_r.x, startnode_r.y };
	position<double> diff_1, diff_2, old_diff(0, 0);
	for (; u < 1.; u += du)
	{
		auto result = bspline.evaluate(u).result();
		diff_1.reset(result[0] - old_tmp[0], result[1] - old_tmp[1]);
		diff_2.reset(diff_1.x - old_diff.x, diff_1.y - old_diff.y);
		double theta = atan2(diff_1.y, diff_1.x);
		//outfile << result[0] << "," << result[1] << "," << theta << endl;
		s += sqrt(pow(result[0] - old_tmp[0], 2.) + pow(result[1] - old_tmp[1], 2.));
		old_tmp[0] = result[0];          old_tmp[1] = result[1];
		old_diff = diff_1;
		if (s < SAFESTEP - ERROR_SAFE)
			continue;

		if (iscollision(result[0], result[1], theta))
		{
			issafe = 1;
			break;
		}
		double k = (diff_1.x*diff_2.y - diff_2.x*diff_1.y) / pow(sqrt(pow(diff_1.x, 2.) + pow(diff_2.y, 2.)), 1.5);

		le_tmp += s;
		s = 0;
		count++;
		if (count == time)
		{
			*le = le_tmp;
			//path->emplace_back(old_node);
			count = 0;
		}
	}

	//将经过碰撞检测后的路径上的控制点再转换成相对路径，用于下面的步骤
	orient_trans_global_vehicle(startnode, &ctrl_points);

	//根据u的最终值判断最后一个正确的控制点，插入时不用插入第一个控制点
	int end_safe = (int)std::floor(u*(ctrl_points.size() / 2 - 3)) + 3;
	Vehicle::Node old_node(startnode_r);
	for (int i = 2; i <= end_safe - 1; i += 2)
	{
		path->emplace_back(Vehicle::Node(ctrl_points[2 * i], ctrl_points[2 * i + 1], atan2(ctrl_points[2 * i + 1] - old_node.y, ctrl_points[2 * i] - old_node.x)/* + startnode.theta*/));
		old_node = *path->rbegin();
	}

	//对于issafe为1的情况有两种特殊情况，一种是是最后一个新插入的中点，那么需要将最后一个控制点弹出；一种是没有插入，那么将issafe重设为0（第二种要在第一种之后判断）
	if (issafe == 1)
	{
		if (u >= 1 - 1 / (ctrl_points.size() / 2 - 3))
			path->pop_back();
		if (path->size() == 0)
			issafe = 0;
	}

	return issafe;
}