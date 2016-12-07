#include "Environment.h"

void Environment::EnvironMap::point_construct(const position &point)
{
	int i = (int)std::floor((point.x - range.begin()->x) / interval);
	int j = (int)std::floor((point.y - range.begin()->y) / interval);
	position key = { i*interval + range.begin()->x, j*interval + range.begin()->y };
	auto occupy = environment.find(key);
	occupy->second = 1;
}

//shape=[length,width]
void Environment::EnvironMap::vehicle_construct(const position &center, const position &shape)// view as a rectangle 
{
	for (float i = center.x; i < center.x + shape.x + interval; i += interval)
	{
		for (float j = center.y - shape.y / 2; j < center.y + shape.y / 2 + interval; j += interval)
		{
			position key(i, j);
			point_construct(key);
		}
	}
}

void Environment::EnvironMap::line_construct(const std::vector<position> &points)
{
	point_construct(*points.begin());
	for (auto p = points.begin() + 1; p != points.end(); p++)
	{
		float d = sqrtf(pow(p->x - (p - 1)->x, 2) + pow(p->y - (p - 1)->y, 2));
		if (d < interval)
			point_construct(*p);
		else
		{
			float theta = atan2(p->y - (p - 1)->y, p->x - (p - 1)->x);
			int N = std::floor(d / interval);
			for (int i = 1; i <= N; i++)
				point_construct(position((p - 1)->x + interval*i*cosf(theta), (p - 1)->y + interval*i*sinf(theta)));
			point_construct(*p);
		}
	}
}

void Environment::EnvironMap::guard_construct(const std::vector<position> &points, const float &width)
{
	for (float i = -width / 2; i < width / 2 + interval; i += interval)
	{
		std::vector<position> points_width = points;
		for (auto &p : points_width)
			p.y += i;
		line_construct(points_width);
	}
}

void Environment::EnvironMap::reset()
{
	for (auto i = environment.begin(); i != environment.end(); i++)
		i->second = 0;
}

void Environment::map_construct(EnvironMap *environmap)
{
	// construct guard 
	float lane_center = 5.f;// 2.25f;
	float guard_width = 0.2f;
	std::vector<position> points;
	for (float i = 0.f; i <= 4.f; i += environmap->_interval())
		points.emplace_back(i, lane_center);
	environmap->guard_construct(points, guard_width);
	points.clear();
	for (float i = 12.f; i <= 33.f; i += environmap->_interval())
		points.emplace_back(i, lane_center);
	environmap->guard_construct(points, guard_width);
	//// construct obstacle
	position center(0, -12.25), shape(4, 21);
	environmap->vehicle_construct(center, shape);
	center.x = 0.;  center.y = 16.75;
	environmap->vehicle_construct(center, shape);
	center.x = 12;  center.y = -9.5; shape.x = 21.;
	environmap->vehicle_construct(center, shape);
	center.x = 12.; center.y = 19.5; 
	environmap->vehicle_construct(center, shape);
	center.x = 7.5; center.y = -9.5; 
	shape.x = 1;    shape.y = 21;
	environmap->vehicle_construct(center, shape);
	center.y = 19.5;
	environmap->vehicle_construct(center, shape);
}