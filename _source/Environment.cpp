#include "Environment.h"

void Environment::EnvironMap::point_construct(const position &point)
{
	int i = (int)std::floor((point.x - range.begin()->x) / interval);
	int j = (int)std::floor((point.y - range.begin()->y) / interval);
	position key = { i*interval + range.begin()->x, j*interval + range.begin()->y };
	auto occupy = environment.find(key);
	occupy->second = 1;
}

void Environment::EnvironMap::point_construct(const double &x, const double &y)
{
	int i = (int)std::floor((x - range.begin()->x) / interval);
	int j = (int)std::floor((y - range.begin()->y) / interval);
	position key = { i*interval + range.begin()->x, j*interval + range.begin()->y };
	auto occupy = environment.find(key);
	occupy->second = 1;
}

//shape=[length,width]
void Environment::EnvironMap::vehicle_construct(const position &center, const position &shape)// view as a rectangle 
{
	for (double i = center.x; i < center.x + shape.x + interval; i += interval)
	{
		for (double j = center.y - shape.y / 2; j < center.y + shape.y / 2 + interval; j += interval)
		{
			position key(i, j);
			point_construct(key);
		}
	}
}

//corner/shape=[x,y]  shape=[x,y]正向为正，可负
void Environment::EnvironMap::box_construct(const position &corner, const position &shape)
{
	position center(corner.x, corner.y + 0.5*shape.y);
	if (shape.x < 0)
		center.x -= shape.x;
	//shape.x = std::abs(shape.x);
	vehicle_construct(center, position(std::abs(shape.x), std::abs(shape.y)));
}

void Environment::EnvironMap::line_construct(const std::vector<position> &points)
{
	point_construct(*points.begin());
	for (auto p = points.begin() + 1; p != points.end(); p++)
	{
		double d = sqrt(pow(p->x - (p - 1)->x, 2) + pow(p->y - (p - 1)->y, 2));
		if (d < interval)
			point_construct(*p);
		else
		{
			double theta = atan2(p->y - (p - 1)->y, p->x - (p - 1)->x);
			int N = (int)std::floor(d / interval);
			for (int i = 1; i <= N; i++)
				point_construct(position((p - 1)->x + interval*i*cos(theta), (p - 1)->y + interval*i*sin(theta)));
			point_construct(*p);
		}
	}
}

void Environment::EnvironMap::line_construct(const position &start, const position &end)
{
	double theta = atan2(end.y - start.y, end.x - start.x);
	double dis = sqrt(pow(start.x - end.x, 2) + pow(start.y - end.y, 2));
	for (double i = 0.; i < dis; i += interval)
		point_construct(start.x + i*cos(theta), start.y + i*sin(theta));
}

void Environment::EnvironMap::guard_construct(const std::vector<position> &points, const double &width)
{
	for (double i = -width / 2; i < width / 2 + interval; i += interval)
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
	//construct box
/*	environmap->box_construct(position(0, 0), position(52, 12));
	environmap->box_construct(position(59, 0), position(12, 12));
	environmap->box_construct(position(0, 31.5), position(52, 12));
	environmap->box_construct(position(59, 31.5), position(12, 12));

	// construct guard 
	environmap->box_construct(position(0, 22.5), position(50, 2));
	environmap->box_construct(position(61, 22.5), position(10, 2));

	// construct traffic lines
	environmap->line_construct(position(45, 15.5), position(50, 15.5));
	environmap->line_construct(position(45, 19), position(50, 19));
	environmap->line_construct(position(45, 28), position(50, 28));
	environmap->line_construct(position(61, 15.5), position(66, 15.5));
	environmap->line_construct(position(61, 19), position(66, 19));
	environmap->line_construct(position(61, 28), position(66, 28));
	environmap->line_construct(position(55.5, 5), position(55.5, 10));
	environmap->line_construct(position(55.5, 33.5), position(55.5, 38.5));*/

	//double lane_center = 5.;// 2.25;
	//double guard_width = 0.2;
	//std::vector<position> points;
	//for (double i = 0.; i <= 4.; i += environmap->_interval())
	//	points.emplace_back(i, lane_center);
	//environmap->guard_construct(points, guard_width);
	//points.clear();
	//for (double i = 12.; i <= 33.; i += environmap->_interval())
	//	points.emplace_back(i, lane_center);
	//environmap->guard_construct(points, guard_width);
	////// construct obstacle
	//position center(0, -12.25), shape(4, 21);
	//environmap->vehicle_construct(center, shape);
	//center.x = 0.;  center.y = 16.75;
	//environmap->vehicle_construct(center, shape);
	//center.x = 12;  center.y = -9.5; shape.x = 21.;
	//environmap->vehicle_construct(center, shape);
	//center.x = 12.; center.y = 19.5; 
	//environmap->vehicle_construct(center, shape);
	//center.x = 7.5; center.y = -9.5; 
	//shape.x = 1;    shape.y = 21;
	//environmap->vehicle_construct(center, shape);
	//center.y = 19.5;
	//environmap->vehicle_construct(center, shape);
	//center.x = 7.5; center.y = 2.25;
	//shape.x = 0.5;    shape.y = 0.5;
	//environmap->vehicle_construct(center, shape);

	/*char dirname[100];
	SYSTEMTIME time;
	::GetLocalTime(&time);
	sprintf_s(dirname, "validate\\");
	system(("md " + std::string(dirname)).c_str());
	char environame[100];
	sprintf_s(environame, "%s\\environment.txt", dirname);
	fstream envir;
	envir.open(environame, std::ios::trunc);
	for (auto &e : *environmap->_environment())
		envir << e.first.x << " " << e.first.y << " " << e.second << std::endl;*/
}