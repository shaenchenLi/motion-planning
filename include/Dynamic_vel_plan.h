#ifndef DYNAMIC_VEL_PLAN
#define DYNAMIC_VEL_PLAN

#include "Trajectory.h"

extern double curr_t, curr_v, curr_a;
extern int entire_traj_num;
extern int u_begin;
extern int local_change;

namespace Trajectory
{
	//计算安全距离
	double _safe_dis(const double v = 0);
	double _safe_time(const double v = 0);

	//判断是否可以起动   2:可以起步，正常行驶  1:可以起步，减速行驶  0:不能起步
	int is_start(vector<State> *entire_traj, const int &curr_u, vector<double> *out_v, vector<double> *out_a);
	//判断是否需要减速   4:正常行驶  3:第二个周期内/*安全距离以外有障碍物*/，减速行驶  2:第二个周期内安全距离以内有障碍物，该段减速，下个周期需要重规划  1:第一个周期内安全距离以外有障碍物，减速行驶  0:第一个周期内安全距离以内有障碍物，避无可避，只能急刹
	int is_slow_down(vector<State> *entire_traj, const int &curr_u, vector<double> *out_v, vector<double> *out_a);
	//用于在减速后需要按照上一次规划的速度行驶时的碰撞检测
	int is_slow_down_2(vector<State> *entire_traj, const int &curr_u, vector<double> *out_v, vector<double> *out_a);

	//按照加速度公式计算速度，u_mid只存在于u1，在u2时会设置不加速
	void _cal_vel(vector<State> *entire_traj, const int &u_index, const int u_add = -1);
	bool _cal_vel(const double &a0, const double &ag, const double &v0, const double &vg, const double &sg, double *a, double *b, double *c, double *tg);
	bool _cal_vel(const double &a0, const double &ag, const double &v0, const double &sg, const double &tg, const double &v_limit, const double &s1, double *a, double *b, double *t1);
	bool _cal_vel(const double &a0, const double &v0, const double &sg, const double &tg, double *a, double *b);
	bool _cal_vel(const double &a0, const double &v0, const double &vg, const double &sg, double *a, double *b, double *tg);
	//根据是否障碍物信息和速度计算规划速度，返回能否计算成功
	bool _vel_plan(vector<State> *entire_traj, const int &u_start, const int &u_colli, const double &t_safe, vector<double> *out_v, vector<double> *out_a);

	//根据碰撞检测结果输出一个规划周期内的规划路径
	int _output(vector<State> *entire_traj, const vector<int> &u_div, const vector<double> &v_div, const int &curr_u, vector<double> *points_x, vector<double> *points_y, vector<double> *points_k, vector<double> *points_v);
}

#endif