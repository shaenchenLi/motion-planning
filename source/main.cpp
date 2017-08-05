//Version 4.1 Liyishan

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>	
#include <stdio.h>
using namespace std::chrono;

#include "Plan.h"

int main()
{
	//Environment::time_EnvironMap dynamic_environmap = Environment::time_EnvironMap(position<double>(XMIN, YMIN), position<double>(XMAX, YMAX), TMAX);

	/*char dirname[100];
	std::ofstream colli, envir;
	sprintf_s(dirname, "..\\..\\validate");
	system(("md " + std::string(dirname)).c_str());
	char envirname[100];
	sprintf_s(envirname, "%s\\dynamic_environment.txt", dirname);
	envir.open(envirname);
	for (auto &e : *dynamic_environmap._environment())
		envir << e.first.x << " " << e.first.y << " " << e.first.t << " " << e.second << endl;
	envir.close();*/
	
	int u = 0;
	double t = 0., v = 5., a = 0.;
	int isLog = 0;

	PLAN_INPUT input;
	input.flag = 2;
	input.road_angle = 0.;
	input.obs = std::make_shared<vector<double>>(vector<double>({ 0., 0., 0. }));
	/*input.XI = std::make_shared<Trajectory::State>(Trajectory::State(0., 0., 0, 0., 0., v, a));
	input.XG = std::make_shared<Trajectory::State>(Trajectory::State(30., 3.5, 0, 0., 0., 10.));*/
	input.XI = std::make_shared<Trajectory::State>(Trajectory::State(0., 0., PI/2, 0., 0., v, a));
	input.XG = std::make_shared<Trajectory::State>(Trajectory::State(-10., 10., PI, 0., 0., v));

	_init(isLog);
	if (isLog == 1)
		path_executed.open(path_executed_name, std::ios::app);
	PLAN_OUTPUT output;
	double t_start = 0.;
	while (1)
	{
		_plan(isLog, input, &output, u, t, v, a, 0);

		int m = output.x->size();
		double s0, t0(0.);
		int u_start = 0;

		if (local_change == 1)
		{
			t_start = 0.;
			u = 0;
			local_change = 0;
		}
		int i = 0;
		for (; i < m - 1; i++)
		{
			s0 = Trajectory::dist((*output.x.get())[i], (*output.y.get())[i], (*output.x.get())[i + 1], (*output.y.get())[i + 1]);
			t0 += 2 * s0 / ((*output.v.get())[i] + (*output.v.get())[i + 1]);
			if (t0 <= t_start)
				u_start = i;
			if (t0 - t_start > PLAN_PERIOD)
			{
				u = std::max(i - 1, u + 1);
				v = (*output.v.get())[u];
				a = (pow((*output.v.get())[i + 1], 2) - pow((*output.v.get())[i], 2)) / (2 * s0);
				break;
			}
		}

		t_start = t0;
		t += PLAN_PERIOD;
		//for (int j = u_start; j < u + 1; j++)
		//	cout << (*output.x.get())[j] << "," << (*output.y.get())[j] << "," << (*output.k.get())[j] << "," << (*output.v.get())[j] << endl;
		if (isLog)
		{
			for (int j = u_start; j < u + 1; j++)
				path_executed << (*output.x.get())[j] << "," << (*output.y.get())[j] << "," << (*output.k.get())[j] << "," << (*output.v.get())[j] << endl;
		}

		if (u + 1 >= output.x.get()->size())
		{
			if (isLog)
				path_executed.close();
			break;
		}
	}

	//system("pause");
}