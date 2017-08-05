#ifndef DYNAMIC_VEL_PLAN
#define DYNAMIC_VEL_PLAN

#include "Trajectory.h"

extern double curr_t, curr_v, curr_a;
extern int entire_traj_num;
extern int u_begin;
extern int local_change;

namespace Trajectory
{
	//���㰲ȫ����
	double _safe_dis(const double v = 0);
	double _safe_time(const double v = 0);

	//�ж��Ƿ������   2:�����𲽣�������ʻ  1:�����𲽣�������ʻ  0:������
	int is_start(vector<State> *entire_traj, const int &curr_u, vector<double> *out_v, vector<double> *out_a);
	//�ж��Ƿ���Ҫ����   4:������ʻ  3:�ڶ���������/*��ȫ�����������ϰ���*/��������ʻ  2:�ڶ��������ڰ�ȫ�����������ϰ���öμ��٣��¸�������Ҫ�ع滮  1:��һ�������ڰ�ȫ�����������ϰ��������ʻ  0:��һ�������ڰ�ȫ�����������ϰ�����޿ɱܣ�ֻ�ܼ�ɲ
	int is_slow_down(vector<State> *entire_traj, const int &curr_u, vector<double> *out_v, vector<double> *out_a);
	//�����ڼ��ٺ���Ҫ������һ�ι滮���ٶ���ʻʱ����ײ���
	int is_slow_down_2(vector<State> *entire_traj, const int &curr_u, vector<double> *out_v, vector<double> *out_a);

	//���ռ��ٶȹ�ʽ�����ٶȣ�u_midֻ������u1����u2ʱ�����ò�����
	void _cal_vel(vector<State> *entire_traj, const int &u_index, const int u_add = -1);
	bool _cal_vel(const double &a0, const double &ag, const double &v0, const double &vg, const double &sg, double *a, double *b, double *c, double *tg);
	bool _cal_vel(const double &a0, const double &ag, const double &v0, const double &sg, const double &tg, const double &v_limit, const double &s1, double *a, double *b, double *t1);
	bool _cal_vel(const double &a0, const double &v0, const double &sg, const double &tg, double *a, double *b);
	bool _cal_vel(const double &a0, const double &v0, const double &vg, const double &sg, double *a, double *b, double *tg);
	//�����Ƿ��ϰ�����Ϣ���ٶȼ���滮�ٶȣ������ܷ����ɹ�
	bool _vel_plan(vector<State> *entire_traj, const int &u_start, const int &u_colli, const double &t_safe, vector<double> *out_v, vector<double> *out_a);

	//������ײ��������һ���滮�����ڵĹ滮·��
	int _output(vector<State> *entire_traj, const vector<int> &u_div, const vector<double> &v_div, const int &curr_u, vector<double> *points_x, vector<double> *points_y, vector<double> *points_k, vector<double> *points_v);
}

#endif