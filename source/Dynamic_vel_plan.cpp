#include "Dynamic_vel_plan.h"

static vector<double> v_precal, a_precal;
static vector<double> v_precal_slow, a_precal_slow;
static int is_cal_vel = 0; //0：不用计算速度  1：使用u_start计算速度  2：直接在后面添加速度  当1和2同时出现，直接用1
static vector<int> u_div;
static vector<double> v_div;

double curr_v(0.), curr_a(0.), curr_t(0.);
int entire_traj_num = 0;
int u_begin = 0; //用于标识速度计算在整个entire_traj中的起点
int local_change = 0; //用于表示u_begin是否需要更换

double Trajectory::_safe_dis(const double v)
{
	double safe_dis = STATIC_SAFE_LENGTH + SAFE_LENGTH_MARGIN;

	if (v != 0)
		safe_dis = pow(v, 2) / (2 * std::abs(AMIN)) + SAFE_LENGTH_MARGIN;
	
	return safe_dis;
}

double Trajectory::_safe_time(const double v)
{
	double safe_time = 2*PLAN_PERIOD;

	if (v != 0)
		safe_time = std::abs(v / AMIN) + ERROR_T;

	return safe_time;
}

//计算预设速度段
bool Trajectory::_cal_vel(const double &a0, const double &ag, const double &v0, const double &vg, const double &sg, double *a, double *b, double *c, double *tg)
{
	bool result = true;
	double t = 0., a_tmp = a0, b_tmp = 0., c_tmp = 0.;

	if (v0 == vg&&v0 == 0)
	{
		*tg = 6 * sg / (3.2*AMAX);
		*b = -6 * sg / pow(*tg, 3);
		*a = -(*tg)*(*b);
		*c = 0;
		return result;
	}

	//利用三次函数计算速度v=v0+a*t+b*t^2+c*t^3
	double A = (a0 - ag) / 3;
	double B = 2 * (v0 + vg);
	double C = -4 * sg;
	
	double delta = pow(B, 2) - 4 * A*C;
	if (delta < 0)
		result = false;
	else
	{
		vector<double> t_candi;
		if (A == 0)
			t_candi.push_back(-C / B);
		else
		{
			double t1 = (-B - sqrt(delta)) / (2 * A);
			double t2 = (-B + sqrt(delta)) / (2 * A);
			t_candi.push_back(t1);
			t_candi.push_back(t2);
		}

		for (auto i = t_candi.begin(); i != t_candi.end(); i++)
		{
			double t_tmp = (*i);
			if (t_tmp <= 0)
				continue;

			c_tmp = (ag*t_tmp - a0*t_tmp - 2 * vg + 2 * v0 + 2 * a0*t_tmp) / pow(t_tmp, 3);
			b_tmp = (ag - a0 - 3 * c_tmp*pow(t_tmp, 2)) / (2 * t_tmp);

			//判断整个过程中加速度是否超届
			vector<double> ae = { a0, ag };			
			double te = -b_tmp / (3 * c_tmp);
			if (te > 0 && te < t_tmp)
				ae.push_back(a0 + 2 * b_tmp*te + 3 * c_tmp*pow(te, 2));
			if (*std::min_element(ae.begin(), ae.end()) < AMIN || *std::max_element(ae.begin(), ae.end()) > AMAX)
				continue;

			//判断整个过程中减速度是否超届
			vector<double> ve = { v0, vg };
			delta = pow(2 * b_tmp, 2) - 4 * a0 * 3 * c_tmp;
			if (delta > 0)
			{
				te = (-2 * b_tmp - sqrt(delta)) / (6 * c_tmp);
				if (te > 0 && te < t_tmp)
					ve.push_back(v0 + a0*te + b_tmp*pow(te, 2) + c_tmp*pow(te, 3));

				te = (-2 * b_tmp + sqrt(delta)) / (6 * c_tmp);
				if (te > 0 && te < t_tmp)
					ve.push_back(v0 + a0*te + b_tmp*pow(te, 2) + c_tmp*pow(te, 3));
			}
			if (*std::min_element(ve.begin(), ve.end()) < 0 || *std::max_element(ve.begin(), ve.end()) > VMAX)
				continue;

			t = t_tmp;
			*a = a_tmp;
			*b = b_tmp;
			*c = c_tmp;
			*tg = t_tmp;
			break;
		}

		if (t == 0.)
			result = false;
	}

	//如果保证边界条件不能够计算成功，那么则省略边界条件，直接计算速度
	if (result == false)
	{
		a_tmp = (pow(vg, 2) - pow(v0, 2)) / (2 * sg);
		if (a_tmp > AMIN&&a_tmp < AMAX)
		{
			result = true;
			*a = a_tmp;
			*b = *c = 0;
			*tg = (vg - v0) / a_tmp;
		}
		else
		{
			if (vg > v0)
			{
				*a = AMAX;
				*b = *c = 0;
				*tg = (vg - v0) / a_tmp;
			}
			else
			{
				*a = AMIN;
				*b = *c = 0;
				*tg = (vg - v0) / a_tmp;
			}
		}
	}
	return result;
}

//计算发生碰撞时的速度段
bool Trajectory::_cal_vel(const double &a0, const double &ag, const double &v0, const double &sg, const double &tg, const double &v_limit, const double &s1, double *a, double *b, double *t1)
{
	//利用二次函数计算速度v=a+b*t+c*t^2
	bool result = true;
	double t = 0., a_tmp = a0, b_tmp = 0.;

	//A*t1^2+B*t1+C=0
	double A = ag / 3 + a0 / 6;
	double B = -0.5*(a0 + ag)*tg;
	double C = sg - v0*tg;

	double delta = pow(B, 2) - 4 * A*C;
	if (delta < 0)
		result = false;
	else
	{
		vector<double> t_candi;
		if (A == 0)
			t_candi.push_back(-C / B);
		else
		{
			double t1 = (-B - sqrt(delta)) / (2 * A);
			double t2 = (-B + sqrt(delta)) / (2 * A);
			t_candi.push_back(t1);
			t_candi.push_back(t2);
		}

		for (auto i = t_candi.begin(); i != t_candi.end(); i++)
		{
			double t_tmp = (*i);
			if (t_tmp <= 0)
				continue;

			b_tmp = (ag - a0) / (2 * t_tmp);

			double s = v0*t_tmp + 0.5*a_tmp*pow(t_tmp, 2) + b_tmp*pow(t_tmp, 3) / 3;
			double vg;
			if (s < s1)
			{
				vg = (sg - s) / (tg - t_tmp);
				//if (vg<0 || vg>v_limit)
					//continue;
			}
			else
			{
				ArrayXXd times = VectorXd::LinSpaced(100, 0., t_tmp);
				ArrayXXd lengths = times*(v0 + times*(A / 2. + times*(B / 3. + times*C / 4)));
				int j = 0;
				while (j + 1 < 100 && s1 >= lengths(j, 0))
					j++;
				double t1_tmp = times(j - 1, 0) + (s - lengths(j - 1, 0))*(times(j, 0) - times(j - 1, 0)) / (lengths(j, 0) - lengths(j - 1, 0));
				vg = v0 + a_tmp*t1_tmp + b_tmp*pow(t1_tmp, 2);
				//if (vg<0 || vg>v_limit)
					//continue;
			}

			//判断整个过程中速度是否超届
			vector<double> ve = { v0, vg };
			double te = -a_tmp / (2 * b_tmp);
			if (te > 0 && te < t_tmp)
				ve.push_back(v0 + a_tmp*te + b_tmp*pow(te, 2));
			if (*std::min_element(ve.begin(), ve.end()) < 0 || *std::max_element(ve.begin(), ve.end()) > v_limit)
				continue;

			t = t_tmp;
			*a = a_tmp;
			*b = b_tmp;
			*t1 = t_tmp;
			break;
		}

		if (t == 0.)
			result = false;
	}

	if (result == false)
	{
		a_tmp = 2 * (sg - v0*tg) / pow(tg, 2);
		if (a_tmp > AMIN && a_tmp < AMAX)
		{
			result = true;
			*a = a_tmp;
			*t1 = std::min(tg, std::abs(v0 / a_tmp));
		}
		else
		{
			*a = AMIN;
			*t1 = tg;
		}		

		*b = 0;
	}
	return result;
}

bool Trajectory::_cal_vel(const double &a0, const double &v0, const double &sg, const double &tg, double *a, double *b)
{
	bool result = true;
	double t = 0., a_tmp = a0, b_tmp = 0.;

	b_tmp = (sg - v0*tg - 0.5*a0*pow(tg, 2)) / (pow(tg, 3) / 3);
	if (a0 + 2 * b_tmp*tg<AMIN || a0 + 2 * b_tmp*tg>AMAX)
		result = false;
	else
	{
		double vg = v0 + a_tmp*tg + b_tmp*pow(tg, 2);

		vector<double> ve = { v0, vg };
		double te = -a_tmp / (2 * b_tmp);
		if (te > 0 && te < tg)
			ve.push_back(v0 + a_tmp*te + b_tmp*pow(te, 2));
		if (*std::min_element(ve.begin(), ve.end()) < 0 || *std::max_element(ve.begin(), ve.end()) > VMAX)
			result = false;
		else
		{
			*a = a_tmp;
			*b = b_tmp;
		}
	}

	if (result == false)
	{
		a_tmp = 2 * (sg - v0*tg) / pow(tg, 2);
		if (a_tmp > AMIN && a_tmp < AMAX)
		{
			result = true;
			*a = a_tmp;
		}
		else
		{
			*a = AMIN;
		}

		*b = 0;
	}
	return result;
}

bool Trajectory::_cal_vel(const double &a0, const double &v0, const double &vg, const double &sg, double *a, double *b, double *tg)
{
	bool result = true;
	double t = 0., a_tmp = a0, b_tmp = 0.;

	double A = a0 / 6;
	double B = (2 * v0 + vg) / 3;
	double C = -sg;

	double delta = pow(B, 2) - 4 * A*C;
	if (delta < 0)
		result = false;
	else
	{
		vector<double> t_candi;
		if (A == 0)
			t_candi.push_back(-C / B);
		else
		{
			double t1 = (-B - sqrt(delta)) / (2 * A);
			double t2 = (-B + sqrt(delta)) / (2 * A);
			t_candi.push_back(t1);
			t_candi.push_back(t2);
		}

		for (auto i = t_candi.begin(); i != t_candi.end(); i++)
		{
			double t_tmp = (*i);
			if (t_tmp <= 0)
				continue;

			b_tmp = (vg - v0 - a_tmp*t_tmp) / pow(t_tmp, 2);

			double ag = a_tmp + 2 * b_tmp*t_tmp;
			if (ag < AMIN || ag > AMAX)
				continue;

			//判断整个过程中减速度是否超届
			vector<double> ve = { v0, vg };
			double te = -B / (2 * A);
			if (te > 0 && te < t_tmp)
				ve.push_back(v0 + a_tmp*te + b_tmp*pow(te, 2));
			if (*std::min_element(ve.begin(), ve.end()) < 0 || *std::max_element(ve.begin(), ve.end()) > VMAX)
				continue;

			t = t_tmp;
			*a = a_tmp;
			*b = b_tmp;
			*tg = t_tmp;
			break;
		}

		if (t == 0.)
			result = false;
	}

	if (result == false)
	{
		a_tmp = (pow(vg, 2) - pow(v0, 2)) / (2 * sg);
		if (a_tmp > AMIN&&a_tmp < AMAX)
		{
			result = true;
			*a = a_tmp;
			*b = 0;
			*tg = (vg - v0) / a_tmp;
		}
		else
		{
			if (vg > v0)
				*a = AMAX;
			else
				*a = AMIN;
			*b = 0;
			*tg = (vg - v0) / a_tmp;
		}
	}

	return result;
}

//计算从curr_u开始的所有速度，若curr_u为-1，说明是在原有的v_precal上添加，第一次进来curr_u为0，不是-1
void Trajectory::_cal_vel(vector<State> *entire_traj, const int &u_index, const int u_add)
{
	double k1, k2, s, s0, v0, a0, v_goal, v, a, t0;
	int u, u_goal;

	if (u_index == -1)
	{
		if (u_add == -1)
			u = entire_traj->size() - POINTS_NUM;
		else
			u = u_add;
		s0 = (*entire_traj)[u]._s();
		v0 = *v_precal.rbegin();
		a0 = *a_precal.rbegin();
	}
	else
	{
		v_precal.clear();
		a_precal.clear();
		u = u_index;
		s0 = (*entire_traj)[u_index]._s();
		v0 = curr_v;
		a0 = curr_a;
	}

	int N = u_div.size();
	int index = 0;
	for (; index < N; index++)
	{
		if (u < u_div[index])
			break;
	}

	int u_flag = (int)(index % 3);
	ArrayXXd times, lengths;
	double A, B, C, T;
	int num, j;

	while (u < entire_traj_num)
	{
		switch (u_flag)
		{
		case 0: //此时位于加速阶段，尽可能到达目标速度
			v_goal = v_div[index];
			u_goal = u_div[index];
			
			s = (*entire_traj)[u_goal]._s() - s0;
			_cal_vel(a0, 0, v0, v_goal, s, &A, &B, &C, &T);
			
			num = u_goal - u;
			times = VectorXd::LinSpaced(num, 0., T);
			lengths = times*(v0 + times*(A / 2. + times*(B / 3. + times*C / 4)));

			j = 0;
			for (; u <= u_goal; u++)
			{
				s = (*entire_traj)[u]._s() - s0;
				while (j + 1 < num && s >= lengths(j, 0))
					j++;
				t0 = times(j - 1, 0) + (s - lengths(j - 1, 0))*(times(j, 0) - times(j - 1, 0)) / (lengths(j, 0) - lengths(j - 1, 0));

				v = v0 + A*t0 + B*pow(t0, 2) + C*pow(t0, 3);
				a = A + 2*B*t0 + 3*C*pow(t0, 2);
				v_precal.push_back(v);
				a_precal.push_back(a);
			}

			s0 = (*entire_traj)[u - 1]._s();
			v0 = *v_precal.rbegin();
			a0 = *a_precal.rbegin();
			++index;

		case 1: //处于保持阶段，如果速度为保持车速就保持允许，否则变更到保持车速
			v_goal = v_div[index];
			u_goal = u_div[index];

			if (v0 == v_goal)
			{
				for (; u <= u_goal; u++)
				{
					v_precal.push_back(v_goal);
					a_precal.push_back(0.);
				}
			}
			else
			{
				s = (*entire_traj)[u_goal]._s() - s0;
				_cal_vel(a0, v0, v_goal, s, &A, &B, &T);

				num = u_goal - u;
				times = VectorXd::LinSpaced(num, 0., T);
				lengths = times*(v0 + times*(A / 2. + times*B / 3.));

				j = 0;
				for (; u <= u_goal; u++)
				{
					s = (*entire_traj)[u]._s() - s0;
					while (j + 1 < num && s >= lengths(j, 0))
						j++;
					t0 = times(j - 1, 0) + (s - lengths(j - 1, 0))*(times(j, 0) - times(j - 1, 0)) / (lengths(j, 0) - lengths(j - 1, 0));

					v = v0 + A*t0 + B*pow(t0, 2);
					a = A + 2 * B*t0;
					v_precal.push_back(v);
					a_precal.push_back(a);
				}
			}

			s0 = (*entire_traj)[u - 1]._s();
			v0 = *v_precal.rbegin();
			a0 = *a_precal.rbegin();
			++index;

		case 2: //到达目标车速阶段，尽力到达目标车速，最后0.01*N个点保持匀速
			v_goal = v_div[index];
			u_goal = u_div[index];

			s = (*entire_traj)[u_goal]._s() - s0;
			_cal_vel(a0, 0, v0, v_goal, s, &A, &B, &C, &T);

			num = u_goal - u;
			times = VectorXd::LinSpaced(num, 0., T);
			lengths = times*(v0 + times*(A / 2. + times*(B / 3. + times*C / 4)));

			j = 0;
			for (; u <= u_goal; u++)
			{
				s = (*entire_traj)[u]._s() - s0;
				while (j + 1 < num && s >= lengths(j, 0))
					j++;
				t0 = times(j - 1, 0) + (s - lengths(j - 1, 0))*(times(j, 0) - times(j - 1, 0)) / (lengths(j, 0) - lengths(j - 1, 0));

				v = v0 + A*t0 + B*pow(t0, 2) + C*pow(t0, 3);
				a = A + 2 * B*t0 + 3 * C*pow(t0, 2);
				v_precal.push_back(v);
				a_precal.push_back(a);
			}

			s0 = (*entire_traj)[u - 1]._s();
			v0 = *v_precal.rbegin();
			a0 = *a_precal.rbegin();
			++index;
		}
		u_flag = (int)(index % 3);		
	}
}

//curr_u表示输出路上的index，真正在entire_traj中的index需要加上u_begin
int Trajectory::is_start(vector<State> *entire_traj, const int &curr_u, vector<double> *out_v, vector<double> *out_a)
{
	//静止的safe_length为定值
	double safe_dis = _safe_dis();

	int u_index = curr_u + u_begin; //现在走到entire_traj中的第u_index个点

	double s;
	double s0 = (*entire_traj)[u_index]._s();
	int u = u_index;
	int u_end = entire_traj->size();// std::min(curr_u + OUTPUT_NUM, POINTS_NUM);

	//检测静止状态下safe_length内是否有障碍物，有则不能起动，没有则可以起动
	bool is_safe = true;
	for (; u < u_end; u++)
	{
		s = (*entire_traj)[u]._s() - s0;
		if (s > safe_dis)
			break;

		Vehicle::Node node = *(*entire_traj)[u]._node();
		//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //坐标转换到车辆坐标便于进行碰撞检测

		if (is_dynamic_collision(node, curr_t))
		{
			is_safe = false;
			break;
		}
	}
	if (is_safe == false)
		return 0;

	//检测起动后两个规划周期内是否有障碍物，有则不能启动
	int is_slow = is_slow_down(entire_traj, curr_u, out_v, out_a);
	
	if (is_slow == 3 || is_slow == 4)
		return 1;
	else
		return 0;
}

//curr_u表示输出路上的index，真正在entire_traj中的index需要加上u_begin
int Trajectory::is_slow_down(vector<State> *entire_traj, const int &curr_u, vector<double> *out_v, vector<double> *out_a)
{
	int u_index = u_begin + curr_u;
	int v_index = curr_u;

	//判断是否需要计算速度，需要则计算，从u_index处开始重新计算

	if (is_cal_vel == 1 || entire_traj_num == 0)
	{
		//u_begin += curr_u;
		//u_index = u_begin;
		v_index = 0;
		entire_traj_num = entire_traj->size();
		_cal_vel(entire_traj, u_index);
	}
	else //附加速度
	{
		if (entire_traj_num < entire_traj->size())
		{
			is_cal_vel = 2;
			entire_traj_num = entire_traj->size();
			_cal_vel(entire_traj, -1);
		}
	}		

	//进行第一个周期内的判断，判断是否需要重规划
	int u = u_index;
	double safe_dis = _safe_dis(curr_v);
	double safe_time = _safe_time(curr_v);

	double s0 = (*entire_traj)[u]._s();
	int u_end = entire_traj->size(); // std::min(curr_u + OUTPUT_NUM, POINTS_NUM);
	int u_length = u_end - u;

	int is_slow = 0;
	int is_safe = 2; //2：正常（无障碍物）  1：减速或重规（safe_obs外有障碍物）  0：急刹

	//检测起动后一个规划周期内是否有障碍物，有则需要重规划
	//按照预设速度进行碰撞检测，判断是否安全
	double t(curr_t), dt(0);//t没有必要存储？
	if (is_dynamic_collision(map_origin, t))
		return 0;
	int i = 1;
	for (; i < u_length; i++)
	{
		dt = ((*entire_traj)[i + u]._s() - (*entire_traj)[i + u - 1]._s()) * 2 / (v_precal[i + v_index] + v_precal[i - 1 + v_index]);
		t += dt;
		if (t - curr_t > safe_time)
			break;

		Vehicle::Node node = *(*entire_traj)[i + u]._node();
		//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //坐标转换到车辆坐标便于进行碰撞检测
		if (is_dynamic_collision(node, t))
		{
			if ((*entire_traj)[i + u]._s() - s0 > safe_dis + ERROR_BRAKE)
				is_safe = 1;
			else
				is_safe = 0;
			break;
		}
	}

	if (is_safe != 2)
		is_slow = is_safe;  //无法避障，尽力刹车
	if (i >= u_length - 1) //车辆已经到达终点
		is_slow = 4;

	//车辆在下一个规划周期还会继续走，因此需要提前检测下一个周期，如果安全正常行驶，不安全提前减速
	//首先计算该规划周期结束时的速度所对应的安全距离
	int j = 1;
	if (is_safe == 2 && is_slow != 4)
	{
		u += i;
		v_index += i;
		safe_dis = _safe_dis(v_precal[v_index]);
		safe_time = _safe_time(v_precal[v_index]);
		double t_add = t;

		s0 = (*entire_traj)[u]._s();
		u_length = u_end - u;

		Vehicle::Node node = *(*entire_traj)[u]._node();
		//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //坐标转换到车辆坐标便于进行碰撞检测
		if (is_dynamic_collision(node, t))
			is_safe = 0;
		else
		{
			for (; j < u_length; j++)
			{
				dt = ((*entire_traj)[j + u]._s() - (*entire_traj)[j + u - 1]._s()) * 2 / (v_precal[j + v_index] + v_precal[j - 1 + v_index]);
				t += dt;
				if (t - curr_t - t_add > safe_time)
					break;

				Vehicle::Node node = *(*entire_traj)[j + u]._node();
				//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //坐标转换到车辆坐标便于进行碰撞检测
				if (is_dynamic_collision(node, t))
				{
					if ((*entire_traj)[j + u]._s() - s0 > safe_dis + ERROR_BRAKE)
						is_safe = 1;
					else
						is_safe = 0;
					break;
				}
			}
		}

		if (is_safe == 2 || j >= u_length - 1)	//车辆已经到达终点或者两个时间周期内的safe_dis都没有障碍物
			is_slow = 4;
		else
		{
			if (is_safe == 0) //第二个周期内safe_dis内有障碍物，下一段重规
				is_slow = 2;
			else if (is_safe == 1) //第二个周期内safe_dis外有障碍物，从现在开始减速
				is_slow = 3;
		}
	}

	if (is_slow != 4 && is_cal_vel == 1)
		is_slow = is_slow_down_2(entire_traj, curr_u, out_v, out_a);
	else
	{
		out_v->clear();				out_a->clear();
		int u_colli, num;
		double t_safe;
		Vehicle::Node node;
		switch (is_slow)
		{
		case 4:
			num = (int)v_precal.size();
			out_v->resize(num);			out_a->resize(num);
			memcpy(&(*out_v)[0], &v_precal[0], sizeof(double)*num);
			memcpy(&(*out_a)[0], &a_precal[0], sizeof(double)*num);
			if (is_cal_vel == 1)
			{
				local_change = 1;
				u_begin = u_index;
				is_cal_vel = 0;
			}
			break;
		case 3:
			u_begin = u_index;

			is_cal_vel = 1;
			local_change = 1;

			u_colli = u + j;

			node = *(*entire_traj)[u_colli]._node();
			//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //坐标转换到车辆坐标便于进行碰撞检测

			if (_safe_time_point(node, t, &t_safe, curr_t + std::max(2 * safe_time, 3 * PLAN_PERIOD)))
			{
				if (!_vel_plan(entire_traj, u_index, u_colli, t_safe, out_v, out_a))
					is_slow = 0;
			}
			else
				is_slow = 0;
			break;
		case 2:
			u_begin = u_index;

			is_cal_vel = 1;
			local_change = 1;

			u_colli = u + j;

			node = *(*entire_traj)[u_colli]._node();
			//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //坐标转换到车辆坐标便于进行碰撞检测

			if (_safe_time_point(node, t, &t_safe, curr_t + std::max(2 * safe_time, 3 * PLAN_PERIOD)))
			{
				if (!_vel_plan(entire_traj, u_index, u_colli, t_safe, out_v, out_a))
					is_slow = 2;
				else
					is_slow = 3;
			}
			else
				is_slow = 0;
			break;
		case 1:
			u_begin = u_index;

			is_cal_vel = 1;
			local_change = 1;

			u_colli = u + i;

			node = *(*entire_traj)[u_colli]._node();
			//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //坐标转换到车辆坐标便于进行碰撞检测

			if (_safe_time_point(node, t, &t_safe, curr_t + std::max(2 * safe_time, 3 * PLAN_PERIOD)))
			{
				if (!_vel_plan(entire_traj, u_index, u_colli, t_safe, out_v, out_a))
					is_slow = 0;
			}
			else
				is_slow = 0;
			break;
		default:
			break;
		}

		//急刹
		if (is_slow == 0)
		{
			is_cal_vel = 0;
			local_change = 1;

			out_v->clear();				out_a->clear();

			double s1 = (*entire_traj)[u_index]._s();
			double v0 = curr_v;
			double v(v0), a(AMIN), s2;
			out_v->push_back(v);		out_a->push_back(a);
			for (int u = u_index + 1; u < u_end; u++)
			{
				s2 = (*entire_traj)[u]._s();
				v = pow(v0, 2) + 2 * AMIN*(s2 - s1);
				if (v > 0)
				{
					v = sqrt(v);
					out_v->push_back(v);
					out_a->push_back(a);
					s1 = s2;
					v0 = v;
				}
				else
				{
					out_v->push_back(0);
					out_a->push_back(0);
					break;
				}
			}
		}
	}

	return is_slow;
}

int Trajectory::is_slow_down_2(vector<State> *entire_traj, const int &curr_u, vector<double> *out_v, vector<double> *out_a)
{
	int u_index = u_begin + curr_u;
	int v_index = curr_u;
	int vel_add = 0;//判断现在是否有新增加一段的速度

	v_precal.clear();							a_precal.clear();
	v_precal.resize(v_precal_slow.size());		a_precal.resize(a_precal_slow.size());
	memcpy(&v_precal[0], &v_precal_slow[0], sizeof(double)*v_precal_slow.size());
	memcpy(&a_precal[0], &a_precal_slow[0], sizeof(double)*a_precal_slow.size());

	int u = u_index;
	double safe_dis = _safe_dis(curr_v);
	double safe_time = _safe_time(curr_v);

	double s0 = (*entire_traj)[u]._s();
	int u_end = entire_traj->size(); // std::min(curr_u + OUTPUT_NUM, POINTS_NUM);
	int u_length = std::min(u_end - u, (int)v_precal.size() - v_index);

	int is_slow = 0;
	int is_safe = 2; //2：正常（无障碍物）  1：减速或重规（safe_obs外有障碍物）  0：急刹

	//检测起动后一个规划周期内是否有障碍物，有则需要重规划
	//按照预设速度进行碰撞检测，判断是否安全
	double t(curr_t), dt(0);//t没有必要存储？
	if (is_dynamic_collision(map_origin, t))
		return 0;
	int i = 1;
	for (; i < u_length; i++)
	{
		dt = ((*entire_traj)[i + u]._s() - (*entire_traj)[i + u - 1]._s()) * 2 / (v_precal[i + v_index] + v_precal[i - 1 + v_index]);
		t += dt;
		if (t - curr_t > safe_time)
			break;

		Vehicle::Node node = *(*entire_traj)[i + u]._node();
		//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //坐标转换到车辆坐标便于进行碰撞检测
		if (is_dynamic_collision(node, t))
		{
			if ((*entire_traj)[i + u]._s() - s0 > safe_dis + ERROR_BRAKE)
				is_safe = 1;
			else
				is_safe = 0;
			break;
		}
	}

	if (is_safe != 2)
		is_slow = is_safe;  //无法避障，尽力刹车
	else
	{
		if (i >= u_length - 1) //车辆已经到达终点
		{
			if (t - curr_t > PLAN_PERIOD)
			{
				is_cal_vel = 1;
				is_slow = 4;
			}
			else
			{
				_cal_vel(entire_traj, -1, i + u);
				vel_add = 1;
			}
		}
	}

	int j = 1;
	if (is_safe == 2 && is_slow != 4)
	{
		u = i + u_index;
		v_index += i;
		safe_dis = _safe_dis(v_precal[v_index]);
		safe_time = _safe_time(v_precal[v_index]);
		double t_add = t;

		s0 = (*entire_traj)[u]._s();
		u_length = std::min(u_end - u, (int)v_precal.size() - v_index);

		Vehicle::Node node = *(*entire_traj)[u]._node();
		//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //坐标转换到车辆坐标便于进行碰撞检测
		if (is_dynamic_collision(node, t))
			is_safe = 0;
		else
		{
			for (; j < u_length; j++)
			{
				dt = ((*entire_traj)[j + u]._s() - (*entire_traj)[j + u - 1]._s()) * 2 / (v_precal[j + v_index] + v_precal[j - 1 + v_index]);
				t += dt;
				if (t - curr_t - t_add > safe_time)
					break;

				Vehicle::Node node = *(*entire_traj)[j + u]._node();
				//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //坐标转换到车辆坐标便于进行碰撞检测
				if (is_dynamic_collision(node, t))
				{
					if ((*entire_traj)[j + u]._s() - s0 > safe_dis + ERROR_BRAKE)
						is_safe = 1;
					else
						is_safe = 0;
					break;
				}
			}
		}

		if (is_safe == 2 || j >= u_length - 1)	//车辆已经到达终点或者两个时间周期内的safe_dis都没有障碍物
			is_slow = 4;
		else
		{
			if (is_safe == 0) //第二个周期内safe_dis内有障碍物，下一段重规
				is_slow = 2;
			else if (is_safe == 1) //第二个周期内safe_dis外有障碍物，从现在开始减速
				is_slow = 3;
		}
	}

	out_v->clear();				out_a->clear();
	int u_colli, num;
	double t_safe;
	Vehicle::Node node;
	switch (is_slow)
	{
	case 4:
		num = (int)v_precal.size();
		out_v->resize(num);			out_a->resize(num);
		memcpy(&(*out_v)[0], &v_precal[0], sizeof(double)*num);
		memcpy(&(*out_a)[0], &a_precal[0], sizeof(double)*num);

		is_cal_vel = 1;
		local_change = 0;

		break;
	case 3:
		u_begin = u_index;
		if (vel_add == 1)
			u_index += i; //新增加的一段中发生碰撞，那么只重新规划新增加一段的速度

		is_cal_vel = 1;
		local_change = 1;

		u_colli = u + j;

		node = *(*entire_traj)[u_colli]._node();
		//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //坐标转换到车辆坐标便于进行碰撞检测

		if (_safe_time_point(node, t, &t_safe, curr_t + std::max(2 * safe_time, 3 * PLAN_PERIOD)))
		{
			if (!_vel_plan(entire_traj, u_index, u_colli, t_safe, out_v, out_a))
				is_slow = 0;
		}
		else
			is_slow = 0;
		break;
	case 2:
		u_begin = u_index;
		if (vel_add == 1)
			u_index += i; //新增加的一段中发生碰撞，那么只重新规划新增加一段的速度

		is_cal_vel = 1;
		local_change = 1;

		u_colli = u + j;

		node = *(*entire_traj)[u_colli]._node();
		//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //坐标转换到车辆坐标便于进行碰撞检测

		if (_safe_time_point(node, t, &t_safe, curr_t + std::max(2 * safe_time, 3 * PLAN_PERIOD)))
		{
			if (!_vel_plan(entire_traj, u_index, u_colli, t_safe, out_v, out_a))
				is_slow = 2;
			else
				is_slow = 3;
		}
		else
			is_slow = 0;
		break;
	case 1:
		u_begin = u_index;
		if (vel_add == 1)
			u_index += i; //新增加的一段中发生碰撞，那么只重新规划新增加一段的速度

		is_cal_vel = 1;
		local_change = 1;

		u_colli = u + i;

		node = *(*entire_traj)[u_colli]._node();
		//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //坐标转换到车辆坐标便于进行碰撞检测

		if (_safe_time_point(node, t, &t_safe, curr_t + std::max(2 * safe_time, 3 * PLAN_PERIOD)))
		{
			if (!_vel_plan(entire_traj, u_index, u_colli, t_safe, out_v, out_a))
				is_slow = 0;
		}
		else
			is_slow = 0;
		break;
	default:
		break;
	}

	//急刹
	if (is_slow == 0)
	{
		is_cal_vel = 0;
		local_change = 1;

		out_v->clear();				out_a->clear();

		double s1 = (*entire_traj)[u_index]._s();
		double v0 = curr_v;
		double v(v0), a(AMIN), s2;
		out_v->push_back(v);		out_a->push_back(a);
		for (int u = u_index + 1; u < u_end; u++)
		{
			s2 = (*entire_traj)[u]._s();
			v = pow(v0, 2) + 2 * AMIN*(s2 - s1);
			if (v > 0)
			{
				v = sqrt(v);
				out_v->push_back(v);
				out_a->push_back(a);
				s1 = s2;
				v0 = v;
			}
			else
			{
				out_v->push_back(0);
				out_a->push_back(a);
				break;
			}
		}
	}

	if (is_slow == 2)
		local_change = 1;
	else
		local_change = 0;

	return is_slow;
}

//为了便于计算，减速阶段采用三次多项式进行速度规划，规划的输出点终点由规划周期和会发生碰撞的地方共同决定，两者取大
//t_safe可能比两个规划周期要大，但是还是要规划到那个时间
bool Trajectory::_vel_plan(vector<State> *entire_traj, const int &u_start, const int &u_colli, const double &t_safe, vector<double> *out_v, vector<double> *out_a)
{
	int N = u_div.size();
	int u_start_index = 0, u_colli_index = 0;
	for (int u = u_colli; u_colli_index < N; u_colli_index++)
	{
		if (u < u_div[u_colli_index])
			break;
	}
	for (int u = u_start; u_start_index < N; u_start_index++)
	{
		if (u < u_div[u_start_index])
			break;
	}
	
	int u_flag;  //0表示碰撞发生的位置和目前位置在同一段轨迹中，但是目前位置不是轨迹段的第一段，采用二次函数计算速度
				 //1表示不管碰撞发生的位置和目前位置是否在同一段轨迹中，但是目前位置是轨迹段的第一段，采用分段函数计算速度，先减速后匀速
				 //2表示碰撞发生的位置和目前位置不在同一段轨迹中，且目前位置不是轨迹段的第一段，采用匀减速运动，因为路径段不一致
	if ((int)std::floor(u_start_index / 3) == (int)std::floor(u_colli_index / 3))
	{
		int u_start_flag = (int)(u_start_index % 3);
		int u_colli_flag = (int)(u_colli_index % 3);
		if (u_start_flag == 0)
			u_flag = 1;
		else
			u_flag = 0;
	}
	else
	{
		int u_start_flag = (int)(u_start_index % 3);
		if (u_start_flag == 0)
			u_flag = 1;
		else
			u_flag = 2;
	}

	double s0 = (*entire_traj)[u_start]._s();
	double a0 = curr_a;
	double v0 = curr_v; //v0一定大于0
	double safe_time = _safe_time(curr_v);
	double tg = t_safe - curr_t;
	out_v->clear();			out_a->clear();

	ArrayXXd times, lengths;
	bool result;
	if (u_flag == 0)
	{
		double sg = (*entire_traj)[u_colli]._s() - s0;
		double A, B;
		result = _cal_vel(a0, v0, sg, tg, &A, &B);

		//判断完速度可行后，就可以计算速度和加速度
		int N = u_colli - u_start + 10; //暂定N为时间间隔数
		times = VectorXd::LinSpaced(N, 0., tg);
		lengths = times*(v0 + times*(A / 2. + times*B / 3.));

		double t0(0), v, a, s;
		int j = 1, u_end = entire_traj->size();
		for (int u = u_start; u < u_end; u++)
		{
			if (u > u_colli && t0 > tg)
				break;
			s = (*entire_traj)[u]._s() - s0;
			while (j + 1 < N && s >= lengths(j, 0))
				j++;
			t0 = times(j - 1, 0) + (s - lengths(j - 1, 0))*(times(j, 0) - times(j - 1, 0)) / (lengths(j, 0) - lengths(j - 1, 0));

			v = v0 + A*t0 + B*pow(t0, 2);
			a = a0 + 2 * B*t0;
			out_v->push_back(v);
			out_a->push_back(a);
		}
	}
	else if (u_flag == 2)
	{
		double sg = (*entire_traj)[u_colli]._s() - s0;
		tg = std::max(std::max(tg, 3 * PLAN_PERIOD), safe_time);
		double k = 2 * (sg - v0*tg) / pow(tg, 2);
		if (k > AMAX || k < AMIN)
		{
			result = false;
			k = k < 0 ? AMIN : AMAX;
		}

		double t0(0), v, a, s;
		int u_end = entire_traj->size();
		for (int u = u_start; u <= u_end; u++)
		{
			if (u > u_colli && t0 > tg)
				break;

			s = (*entire_traj)[u]._s() - s0;
			v = pow(v0, 2) + 2 * k*s;

			if (v < 0)
			{
				out_v->push_back(0);
				out_a->push_back(0);
			}
			else
			{
				v = sqrt(v);
				out_v->push_back(sqrt(v));
				out_a->push_back(k);
			}
		}
	}
	else
	{
		double s1, sg;
		int u_mid = u_div[(int)std::floor(u_start_index / 3)];
		s1 = (*entire_traj)[u_mid]._s() - s0;
		sg = (*entire_traj)[u_colli]._s() - s0;

		double A, B, t1;
		result = _cal_vel(a0, 0, v0, sg, tg, v_div[u_start_index], s1, &A, &B, &t1);

		int N = u_colli - u_start;
		times = VectorXd::LinSpaced(N, 0., tg);
		lengths = times*(v0 + times*(A / 2. + times*B / 3.));

		tg = std::max(std::max(tg, 3 * PLAN_PERIOD), safe_time);
		double s, v, a, t;
		int j = 0;
		int u = u_start;
		for (; u <= u_colli; u++)
		{
			s = (*entire_traj)[u]._s() - s0; 
			while (j + 1 < N && s >= lengths(j, 0))
				j++;
			t = times(j - 1, 0) + (s - lengths(j - 1, 0))*(times(j, 0) - times(j - 1, 0)) / (lengths(j, 0) - lengths(j - 1, 0));
			if (t > t1)
				break;

			v = v0 + A*t + B*pow(t, 2);
			a = A + 2 * B*t;
			out_v->push_back(v);
			out_a->push_back(a);
		}

		if (v > 0)
		{
			for (; u <= u_colli; u++)
			{
				s = (*entire_traj)[u]._s() - (*entire_traj)[u - 1]._s();
				t += s / v;
				if (t > tg)
					break;
				out_v->push_back(v);
				out_a->push_back(0);
			}
		}
	}
	return result;
}

int Trajectory::_output(vector<State> *entire_traj, const vector<int> &u_d, const vector<double> &v_d, const int &curr_u, vector<double> *points_x, vector<double> *points_y, vector<double> *points_k, vector<double> *points_v)
{
	u_div = u_d;
	v_div = v_d;

	//(*entire_traj)[curr_u]._v(curr_v); //将实时车速返回给要跟踪的下一个点

	int state = 0; //车辆本身的状态  0 正常（通过is_start和is_slow可以直接输出速度）  1 静止  2 急刹  3 下段重规
	double t_start = curr_t;  //车辆作为运动状态开始的时间
	vector<double> out_v, out_a;
	if (curr_v == 0)
	{
		state = 1;  //初始车速为0，因此需要判断车辆是否能够起步，因此变初始化为静止状态
		for (double t = 0; t < PLAN_PERIOD; t += T_INTERVAL)
		{
			t_start = curr_t + t;
			if (is_start(entire_traj, curr_u, &out_v, &out_a))
			{
				state = 0;
				break;
			}
		}
	}
	else
	{
		switch (is_slow_down(entire_traj, curr_u, &out_v, &out_a))
		{
		case 0:
			state = 2;
			break;
		case 2:
			state = 3;
			break;
		default:
			break;
		}
	}

	points_x->clear();
	points_y->clear();
	points_k->clear();
	points_v->clear();
	int is_replan = 0;  //判断下段是否重规
	int output_num = 100; //如果静止，输出的点的个数
	double t_total = 2 * PLAN_PERIOD;  //如果运动，输出两个规划周期内的点的个数
	int N = (int)out_v.size();
	switch (state)
	{
	case 1: //静止
		for (int i = 0; i < output_num; i++)
		{
			double x = (*entire_traj)[u_begin]._node()->x;
			double y = (*entire_traj)[u_begin]._node()->y;
			points_x->push_back(x);
			points_y->push_back(y);
			points_k->push_back(0);
			points_v->push_back(0);
		}
		break;
	case 3:
		is_replan = 1;
	case 0: 
	case 2:
		for (int i = 0; i < N; i++)
		{
			Vehicle::Node node = *(*entire_traj)[u_begin + i]._node();
			points_x->push_back(node.x);
			points_y->push_back(node.y);
			points_k->push_back(node.k);
			points_v->push_back(out_v[i]);
		}
		v_precal_slow.clear();					a_precal_slow.clear();
		v_precal_slow.resize(out_v.size());		a_precal_slow.resize(out_a.size());
		memcpy(&v_precal_slow[0], &out_v[0], sizeof(double)*out_v.size());
		memcpy(&a_precal_slow[0], &out_a[0], sizeof(double)*out_a.size());
		break;
	}

	return is_replan;
}