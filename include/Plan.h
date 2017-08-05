#ifndef PLAN_H
#define PLAN_H

#include <chrono>
using namespace std::chrono;
#include <memory>
using std::shared_ptr;
#include <thread>  
#include <mutex>

#include "collision_check.h"
#include "Dynamic_vel_plan.h"
#include "Force_extend.h"
#include "RRT.h"
#include "Trajectory.h"
#include "Vehicle.h"
#include "show.h"

extern ofstream path_executed;
extern char path_executed_name[100];

extern int length;

struct PLAN_INPUT
{
	shared_ptr<Trajectory::State> XI, XG;
	shared_ptr<vector<double>> obs;
	double road_angle; //全局路径的航向角与车辆航向角的夹角
	int flag;

	PLAN_INPUT()
	{
		XI = std::make_shared<Trajectory::State>();
		XG = std::make_shared<Trajectory::State>();
		obs = std::make_shared<vector<double>>();
		road_angle = 0;
		flag = 0;
	}

	PLAN_INPUT(const PLAN_INPUT &i)
	{
		XI = i.XI;
		XG = i.XG;
		obs = i.obs;
		road_angle = i.road_angle;
		flag = i.flag;
	}
	~PLAN_INPUT() {}

	bool operator==(const PLAN_INPUT &input) const
	{
		bool is_equal = true;

		if (*XI.get() == *input.XI.get() && *XG.get() == *input.XG.get() && flag == input.flag && road_angle == input.road_angle)
		{
			int m = obs->size();
			int n = obs->size();

			if (m != n)
				is_equal = false;
			else
			{
				for (int i = 0; i < n; i++)
					if ((*obs.get())[i] != (*input.obs.get())[i])
					{
						is_equal = false;
						break;
					}
			}
		}
		else
			is_equal = false;

		return is_equal;
	}

	void reset()
	{
		XI.reset();
		XG.reset();
		obs->clear();
		flag = 0;
		road_angle = 0;
	}
};

struct PLAN_OUTPUT
{
	shared_ptr<vector<double>> x, y, v, k;
	int success;
	//以下三个值目前还没有赋值
	int is_local;
	int local_to_local;
	shared_ptr<Trajectory::State> XG_planned;

	PLAN_OUTPUT()
	{
		x = std::make_shared<vector<double>>();
		y = std::make_shared<vector<double>>();
		k = std::make_shared<vector<double>>();
		v = std::make_shared<vector<double>>();
		success = 0;
		local_to_local = 0;
		is_local = 0;
		XG_planned = std::make_shared<Trajectory::State>();
		/*for (int j = 0; j < 4; j++)
			XG_planned.get()[j] = 0;*/
	}
	PLAN_OUTPUT(const PLAN_OUTPUT &o)
	{
		success = o.success;
		is_local = o.is_local;
		local_to_local = o.local_to_local;
		x = o.x;
		y = o.y;
		v = o.v;
		k = o.k;
		XG_planned = o.XG_planned;
	}
	~PLAN_OUTPUT()	{}
};

void _init(const int &isLog);

//只计算规划前的标志位，规划后的标志位如last_flag和search_success在规划后赋值
void is_to_plan();

// 里面的路径或者速度以及标志位都在这个函数中赋值
void _no_local_plan();
void search_path();
void plan_velocity();
void search_path_append();

void _plan(const int &isLog, const PLAN_INPUT &input, PLAN_OUTPUT *output, const int &u, const double &t, const double &v, const double &a, const int &is_new_map);

void _log();

#endif