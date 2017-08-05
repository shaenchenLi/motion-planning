#include "Dynamic_vel_plan.h"

static vector<double> v_precal, a_precal;
static vector<double> v_precal_slow, a_precal_slow;
static int is_cal_vel = 0; //0�����ü����ٶ�  1��ʹ��u_start�����ٶ�  2��ֱ���ں�������ٶ�  ��1��2ͬʱ���֣�ֱ����1
static vector<int> u_div;
static vector<double> v_div;

double curr_v(0.), curr_a(0.), curr_t(0.);
int entire_traj_num = 0;
int u_begin = 0; //���ڱ�ʶ�ٶȼ���������entire_traj�е����
int local_change = 0; //���ڱ�ʾu_begin�Ƿ���Ҫ����

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

//����Ԥ���ٶȶ�
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

	//�������κ��������ٶ�v=v0+a*t+b*t^2+c*t^3
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

			//�ж����������м��ٶ��Ƿ񳬽�
			vector<double> ae = { a0, ag };			
			double te = -b_tmp / (3 * c_tmp);
			if (te > 0 && te < t_tmp)
				ae.push_back(a0 + 2 * b_tmp*te + 3 * c_tmp*pow(te, 2));
			if (*std::min_element(ae.begin(), ae.end()) < AMIN || *std::max_element(ae.begin(), ae.end()) > AMAX)
				continue;

			//�ж����������м��ٶ��Ƿ񳬽�
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

	//�����֤�߽��������ܹ�����ɹ�����ô��ʡ�Ա߽�������ֱ�Ӽ����ٶ�
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

//���㷢����ײʱ���ٶȶ�
bool Trajectory::_cal_vel(const double &a0, const double &ag, const double &v0, const double &sg, const double &tg, const double &v_limit, const double &s1, double *a, double *b, double *t1)
{
	//���ö��κ��������ٶ�v=a+b*t+c*t^2
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

			//�ж������������ٶ��Ƿ񳬽�
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

			//�ж����������м��ٶ��Ƿ񳬽�
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

//�����curr_u��ʼ�������ٶȣ���curr_uΪ-1��˵������ԭ�е�v_precal����ӣ���һ�ν���curr_uΪ0������-1
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
		case 0: //��ʱλ�ڼ��ٽ׶Σ������ܵ���Ŀ���ٶ�
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

		case 1: //���ڱ��ֽ׶Σ�����ٶ�Ϊ���ֳ��پͱ������������������ֳ���
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

		case 2: //����Ŀ�공�ٽ׶Σ���������Ŀ�공�٣����0.01*N���㱣������
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

//curr_u��ʾ���·�ϵ�index��������entire_traj�е�index��Ҫ����u_begin
int Trajectory::is_start(vector<State> *entire_traj, const int &curr_u, vector<double> *out_v, vector<double> *out_a)
{
	//��ֹ��safe_lengthΪ��ֵ
	double safe_dis = _safe_dis();

	int u_index = curr_u + u_begin; //�����ߵ�entire_traj�еĵ�u_index����

	double s;
	double s0 = (*entire_traj)[u_index]._s();
	int u = u_index;
	int u_end = entire_traj->size();// std::min(curr_u + OUTPUT_NUM, POINTS_NUM);

	//��⾲ֹ״̬��safe_length���Ƿ����ϰ���������𶯣�û���������
	bool is_safe = true;
	for (; u < u_end; u++)
	{
		s = (*entire_traj)[u]._s() - s0;
		if (s > safe_dis)
			break;

		Vehicle::Node node = *(*entire_traj)[u]._node();
		//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //����ת��������������ڽ�����ײ���

		if (is_dynamic_collision(node, curr_t))
		{
			is_safe = false;
			break;
		}
	}
	if (is_safe == false)
		return 0;

	//����𶯺������滮�������Ƿ����ϰ������������
	int is_slow = is_slow_down(entire_traj, curr_u, out_v, out_a);
	
	if (is_slow == 3 || is_slow == 4)
		return 1;
	else
		return 0;
}

//curr_u��ʾ���·�ϵ�index��������entire_traj�е�index��Ҫ����u_begin
int Trajectory::is_slow_down(vector<State> *entire_traj, const int &curr_u, vector<double> *out_v, vector<double> *out_a)
{
	int u_index = u_begin + curr_u;
	int v_index = curr_u;

	//�ж��Ƿ���Ҫ�����ٶȣ���Ҫ����㣬��u_index����ʼ���¼���

	if (is_cal_vel == 1 || entire_traj_num == 0)
	{
		//u_begin += curr_u;
		//u_index = u_begin;
		v_index = 0;
		entire_traj_num = entire_traj->size();
		_cal_vel(entire_traj, u_index);
	}
	else //�����ٶ�
	{
		if (entire_traj_num < entire_traj->size())
		{
			is_cal_vel = 2;
			entire_traj_num = entire_traj->size();
			_cal_vel(entire_traj, -1);
		}
	}		

	//���е�һ�������ڵ��жϣ��ж��Ƿ���Ҫ�ع滮
	int u = u_index;
	double safe_dis = _safe_dis(curr_v);
	double safe_time = _safe_time(curr_v);

	double s0 = (*entire_traj)[u]._s();
	int u_end = entire_traj->size(); // std::min(curr_u + OUTPUT_NUM, POINTS_NUM);
	int u_length = u_end - u;

	int is_slow = 0;
	int is_safe = 2; //2�����������ϰ��  1�����ٻ��ع棨safe_obs�����ϰ��  0����ɲ

	//����𶯺�һ���滮�������Ƿ����ϰ��������Ҫ�ع滮
	//����Ԥ���ٶȽ�����ײ��⣬�ж��Ƿ�ȫ
	double t(curr_t), dt(0);//tû�б�Ҫ�洢��
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
		//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //����ת��������������ڽ�����ײ���
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
		is_slow = is_safe;  //�޷����ϣ�����ɲ��
	if (i >= u_length - 1) //�����Ѿ������յ�
		is_slow = 4;

	//��������һ���滮���ڻ�������ߣ������Ҫ��ǰ�����һ�����ڣ������ȫ������ʻ������ȫ��ǰ����
	//���ȼ���ù滮���ڽ���ʱ���ٶ�����Ӧ�İ�ȫ����
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
		//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //����ת��������������ڽ�����ײ���
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
				//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //����ת��������������ڽ�����ײ���
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

		if (is_safe == 2 || j >= u_length - 1)	//�����Ѿ������յ��������ʱ�������ڵ�safe_dis��û���ϰ���
			is_slow = 4;
		else
		{
			if (is_safe == 0) //�ڶ���������safe_dis�����ϰ����һ���ع�
				is_slow = 2;
			else if (is_safe == 1) //�ڶ���������safe_dis�����ϰ�������ڿ�ʼ����
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
			//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //����ת��������������ڽ�����ײ���

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
			//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //����ת��������������ڽ�����ײ���

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
			//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //����ת��������������ڽ�����ײ���

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

		//��ɲ
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
	int vel_add = 0;//�ж������Ƿ���������һ�ε��ٶ�

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
	int is_safe = 2; //2�����������ϰ��  1�����ٻ��ع棨safe_obs�����ϰ��  0����ɲ

	//����𶯺�һ���滮�������Ƿ����ϰ��������Ҫ�ع滮
	//����Ԥ���ٶȽ�����ײ��⣬�ж��Ƿ�ȫ
	double t(curr_t), dt(0);//tû�б�Ҫ�洢��
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
		//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //����ת��������������ڽ�����ײ���
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
		is_slow = is_safe;  //�޷����ϣ�����ɲ��
	else
	{
		if (i >= u_length - 1) //�����Ѿ������յ�
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
		//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //����ת��������������ڽ�����ײ���
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
				//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //����ת��������������ڽ�����ײ���
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

		if (is_safe == 2 || j >= u_length - 1)	//�����Ѿ������յ��������ʱ�������ڵ�safe_dis��û���ϰ���
			is_slow = 4;
		else
		{
			if (is_safe == 0) //�ڶ���������safe_dis�����ϰ����һ���ع�
				is_slow = 2;
			else if (is_safe == 1) //�ڶ���������safe_dis�����ϰ�������ڿ�ʼ����
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
			u_index += i; //�����ӵ�һ���з�����ײ����ôֻ���¹滮������һ�ε��ٶ�

		is_cal_vel = 1;
		local_change = 1;

		u_colli = u + j;

		node = *(*entire_traj)[u_colli]._node();
		//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //����ת��������������ڽ�����ײ���

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
			u_index += i; //�����ӵ�һ���з�����ײ����ôֻ���¹滮������һ�ε��ٶ�

		is_cal_vel = 1;
		local_change = 1;

		u_colli = u + j;

		node = *(*entire_traj)[u_colli]._node();
		//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //����ת��������������ڽ�����ײ���

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
			u_index += i; //�����ӵ�һ���з�����ײ����ôֻ���¹滮������һ�ε��ٶ�

		is_cal_vel = 1;
		local_change = 1;

		u_colli = u + i;

		node = *(*entire_traj)[u_colli]._node();
		//Vehicle::orient_trans_global_vehicle(map_origin, &node);  //����ת��������������ڽ�����ײ���

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

	//��ɲ
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

//Ϊ�˱��ڼ��㣬���ٽ׶β������ζ���ʽ�����ٶȹ滮���滮��������յ��ɹ滮���ںͻᷢ����ײ�ĵط���ͬ����������ȡ��
//t_safe���ܱ������滮����Ҫ�󣬵��ǻ���Ҫ�滮���Ǹ�ʱ��
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
	
	int u_flag;  //0��ʾ��ײ������λ�ú�Ŀǰλ����ͬһ�ι켣�У�����Ŀǰλ�ò��ǹ켣�εĵ�һ�Σ����ö��κ��������ٶ�
				 //1��ʾ������ײ������λ�ú�Ŀǰλ���Ƿ���ͬһ�ι켣�У�����Ŀǰλ���ǹ켣�εĵ�һ�Σ����÷ֶκ��������ٶȣ��ȼ��ٺ�����
				 //2��ʾ��ײ������λ�ú�Ŀǰλ�ò���ͬһ�ι켣�У���Ŀǰλ�ò��ǹ켣�εĵ�һ�Σ������ȼ����˶�����Ϊ·���β�һ��
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
	double v0 = curr_v; //v0һ������0
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

		//�ж����ٶȿ��к󣬾Ϳ��Լ����ٶȺͼ��ٶ�
		int N = u_colli - u_start + 10; //�ݶ�NΪʱ������
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

	//(*entire_traj)[curr_u]._v(curr_v); //��ʵʱ���ٷ��ظ�Ҫ���ٵ���һ����

	int state = 0; //���������״̬  0 ������ͨ��is_start��is_slow����ֱ������ٶȣ�  1 ��ֹ  2 ��ɲ  3 �¶��ع�
	double t_start = curr_t;  //������Ϊ�˶�״̬��ʼ��ʱ��
	vector<double> out_v, out_a;
	if (curr_v == 0)
	{
		state = 1;  //��ʼ����Ϊ0�������Ҫ�жϳ����Ƿ��ܹ��𲽣���˱��ʼ��Ϊ��ֹ״̬
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
	int is_replan = 0;  //�ж��¶��Ƿ��ع�
	int output_num = 100; //�����ֹ������ĵ�ĸ���
	double t_total = 2 * PLAN_PERIOD;  //����˶�����������滮�����ڵĵ�ĸ���
	int N = (int)out_v.size();
	switch (state)
	{
	case 1: //��ֹ
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