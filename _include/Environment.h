#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <unordered_map>
#include <vector>
using std::vector;
using std::unordered_map;

#include "Parameters.h"

namespace Environment
{
	struct position
	{
		double x, y;
		position(const double &a, const double &b) :x(a), y(b) {}
	};

	struct EnvironMap
	{
		struct HashFunc
		{
			size_t operator()(const position &p) const
			{
				return std::hash<double>()(p.x);
			}
		};

		struct Equal
		{
			bool operator()(const position &p1, const position &p2) const
			{
				return p1.x == p2.x && p1.y == p2.y;
			}
		};

		using en_map = unordered_map<position, int, HashFunc, Equal>;

		EnvironMap(const position &xymin, const position &xymax, const double inter = INTERVAL) :interval(inter)
		{
			range = { xymin, xymax };
			int N_x = (int)std::ceil((xymax.x - xymin.x) / interval);
			int N_y = (int)std::ceil((xymax.y - xymin.y) / interval);
			for (int i = 0; i <= N_x; i++)
			{
				for (int j = 0; j <= N_y; j++)
				{
					position key(i*interval + xymin.x, j*interval + xymin.y);
					environment.insert({ key, 0 });
				}
				position key(i*interval + xymin.x, xymax.y);
				environment.insert({ key, 0 });
			}
			environment.insert({ xymax, 0 });
		}

		void point_construct(const position &point);
		void point_construct(const double &x, const double &y);
		void vehicle_construct(const position &center, const position &shape);
		void line_construct(const vector<position> &points); //the interval of points must equal to the map's interval
		void line_construct(const position &start, const position &end);
		void guard_construct(const vector<position> &points, const double &width);
		void box_construct(const position &corner, const position &shape);
		void reset();

		//get data
		vector<position>* _range() { return &range; }
		double _interval() const { return interval; }
		en_map* _environment() { return &environment; }

	private:
		en_map environment;
		double interval;
		vector<position> range;
	};

	void map_construct(EnvironMap *environmap);
}


#endif