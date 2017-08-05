#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <unordered_map>
#include <vector>
using std::vector;
using std::unordered_map;

#include "Vehicle.h"

namespace Environment
{
/******************************************动态环境地图*******************************************/
	struct time_EnvironMap
	{
		struct HashFunc
		{
			size_t operator()(const time_position<int> &p) const
			{
				return std::hash<int>()(p.x);
			}
		};

		struct Equal
		{
			bool operator()(const time_position<int> &p1, const time_position<int> &p2) const
			{
				return p1.x == p2.x && p1.y == p2.y && p1.t == p2.t;
			}
		};

		using time_en_map = unordered_map<time_position<int>, int, HashFunc, Equal>;

		time_EnvironMap(const position<double> &xymin, const position<double> &xymax, const double &tmax, const double inter = INTERVAL, const double t_inter = T_INTERVAL);

		//static obstacles
		void point_construct(const position<double> &point);
		void point_construct(const double &x, const double &y);
		void vehicle_construct(const position<double> &center, const position<double> &shape);
		void box_construct(const position<double> &center, const position<double> &shape);
		void line_construct(const vector<position<double>> &points);
		void line_construct(const position<double> &start, const position<double> &end);
		void guard_construct(const vector<position<double>> &points, const double &width);

		//dynamic obstacles
		void point_construct(const time_position<double> &point);
		void point_construct(const double &x, const double &y, const double &t);
		void vehicle_construct(const position<double> &center, const position<double> &shape, const double &t);
		void vehicle_construct(const position<double> &center, const position<double> &shape, const double &theta, const double &v, const double t0 = 0.);
		void box_construct(const position<double> &center, const position<double> &shape, const double &t);
		void line_construct(const vector<time_position<double>> &points); //the interval of points must equal to the map's interval
		void line_construct(const time_position<double> &start, const time_position<double> &end);
		void guard_construct(const vector<time_position<double>> &points, const double &width);

		void reset();

		//get data
		vector<position<double>>* _range() { return &range; }
		double _interval() const { return interval; }
		double _time_interval() const { return t_interval; }
		time_en_map* _environment() { return &environment; }
		double _t_range() const { return t_range; }

	private:
		time_en_map environment;
		double interval, t_interval;
		double t_range;
		vector<position<double>> range;
	};

	void map_construct(time_EnvironMap *environmap);
}

#endif