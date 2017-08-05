#include "Environment.h"

double eps_map = 1e-6;

/******************************************动态环境地图*******************************************/
Environment::time_EnvironMap::time_EnvironMap(const position<double> &xymin, const position<double> &xymax, const double &tmax, const double inter, const double t_inter)
{
	interval = inter;
	t_interval = t_inter;
	range = { xymin, xymax };
	t_range = tmax;
	int N_x = (int)std::ceil((xymax.x - xymin.x) / interval);
	int N_y = (int)std::ceil((xymax.y - xymin.y) / interval);
	int N_t = (int)std::ceil(tmax / t_inter);
	for (int i = 0; i <= N_x; i++)
		for (int j = 0; j <= N_y; j++)
			for (int t = 0; t <= N_t; t++)
			{
				time_position<int> key(i, j, t);
				environment.insert({ key, 0 });
			}

	map_construct(this);
}

void Environment::time_EnvironMap::point_construct(const position<double> &point)
{
	int i = (int)std::floor((point.x - range.begin()->x) / interval);
	int j = (int)std::floor((point.y - range.begin()->y) / interval);

	if (i*interval + interval - (point.x - range.begin()->x) < eps_map)
		++i;
	if (j*interval + interval - (point.y - range.begin()->y) < eps_map)
		++j;

	int N_t = (int)std::ceil(t_range / t_interval);
	for (int t = 0; t <= N_t; t++)
	{
		time_position<int> key = { i, j, t };
		auto occupy = environment.find(key);
		occupy->second = 1;
	}
}

void Environment::time_EnvironMap::point_construct(const double &x, const double &y)
{
	int i = (int)std::floor((x - range.begin()->x) / interval);
	int j = (int)std::floor((y - range.begin()->y) / interval);

	if (i*interval + interval - (x - range.begin()->x) < eps_map)
		++i;
	if (j*interval + interval - (y - range.begin()->y) < eps_map)
		++j;

	int N_t = (int)std::ceil(t_range / t_interval);
	for (int t = 0; t <= N_t; t++)
	{
		time_position<int> key = { i, j, t };
		auto occupy = environment.find(key);
		occupy->second = 1;
	}
}

void Environment::time_EnvironMap::point_construct(const double &x, const double &y, const double &t)
{
	int i = (int)std::floor((x - range.begin()->x) / interval);
	int j = (int)std::floor((y - range.begin()->y) / interval);
	int k = (int)std::floor(t / t_interval);

	if (i*interval + interval - (x - range.begin()->x) < eps_map)
		++i;
	if (j*interval + interval - (y - range.begin()->y) < eps_map)
		++j;
	if ((k*t_interval + t_interval - t) < eps_map)
		++k;

	time_position<int> key = { i, j, k };
	auto occupy = environment.find(key);
	occupy->second = 2;
}

void Environment::time_EnvironMap::point_construct(const time_position<double> &point)
{
	int i = (int)std::floor((point.x - range.begin()->x) / interval);
	int j = (int)std::floor((point.y - range.begin()->y) / interval);
	int t = (int)std::floor(point.t / t_interval);

	if (i*interval + interval - (point.x - range.begin()->x) < eps_map)
		++i;
	if (j*interval + interval - (point.y - range.begin()->y) < eps_map)
		++j;
	if ((t*t_interval + t_interval - point.t) < eps_map)
		++t;

	time_position<int> key = { i, j, t };
	auto occupy = environment.find(key);
	occupy->second = 2;
}

//shape=[length,width]
void Environment::time_EnvironMap::vehicle_construct(const position<double> &center, const position<double> &shape)
{
	int N_t = (int)std::ceil(t_range / t_interval);
	for (double i = center.x; i < center.x + shape.x + interval; i += interval)
	{
		for (double j = center.y - shape.y / 2; j < center.y + shape.y / 2 + interval; j += interval)
		{
			position<double> key(i, j);
			point_construct(key);
		}
	}
}

//shape=[length,width]  theta为车辆中心线和width的夹角
void Environment::time_EnvironMap::vehicle_construct(const position<double> &center, const position<double> &shape, const double &theta, const double &v, const double t0)
{
	double x, y;

	for (double i = center.x; i < center.x + shape.x + interval; i += interval)
	{
		for (double j = center.y - shape.y / 2; j < center.y + shape.y / 2 + interval; j += interval)
		{
			for (double t = 0; t <= t_range - t0; t += t_interval)
			{
				x = i + v*t*cos(theta);
				y = j + v*t*sin(theta);
				//x = (i - center.x + v*t)*cos(theta) + center.x;
				//y = center.y + (j - center.y)*sin(theta);
				//y = (j - center.y + v*t)*cos(theta) + center.y;
				if (x >= range.rbegin()->x || y >= range.rbegin()->y)
					break;
				if (x <= range.begin()->x || y <= range.begin()->y)
					break;

				time_position<double> key(x, y, t + t0);
				point_construct(key);
			}
		}
	}
}

void Environment::time_EnvironMap::vehicle_construct(const position<double> &center, const position<double> &shape, const double &t)
{
	for (double i = center.x; i < center.x + shape.x + interval; i += interval)
	{
		for (double j = center.y - shape.y / 2; j < center.y + shape.y / 2 + interval; j += interval)
		{
			time_position<double> key(i, j, t);
			point_construct(key);
		}
	}
}

//corner/shape=[x,y]  shape=[x,y]正向为正，可负
void Environment::time_EnvironMap::box_construct(const position<double> &corner, const position<double> &shape)
{
	position<double> center(corner.x, corner.y + 0.5*shape.y);
	if (shape.x < 0)
		center.x -= shape.x;
	vehicle_construct(center, position<double>(std::abs(shape.x), std::abs(shape.y)));
}

//corner/shape=[x,y]  shape=[x,y]正向为正，可负
void Environment::time_EnvironMap::box_construct(const position<double> &corner, const position<double> &shape, const double &t)
{
	position<double> center(corner.x, corner.y + 0.5*shape.y);
	if (shape.x < 0)
		center.x -= shape.x;
	//shape.x = std::abs(shape.x);
	vehicle_construct(center, position<double>(std::abs(shape.x), std::abs(shape.y)), t);
}

void Environment::time_EnvironMap::line_construct(const std::vector<position<double>> &points)
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
				point_construct((p - 1)->x + interval*i*cos(theta), (p - 1)->y + interval*i*sin(theta));
			point_construct(*p);
		}
	}
}

void Environment::time_EnvironMap::line_construct(const std::vector<time_position<double>> &points)
{
	point_construct(*points.begin());
	for (auto p = points.begin() + 1; p != points.end(); p++)
	{
		double d = sqrt(pow(p->x - (p - 1)->x, 2) + pow(p->y - (p - 1)->y, 2));
		double dt = p->t - (p - 1)->t;
		if (d < interval&&dt < t_interval)
			point_construct(*p);
		else
		{
			double theta = atan2(p->y - (p - 1)->y, p->x - (p - 1)->x);
			double v = d / dt;
			int N = (int)std::floor(dt / t_interval);
			double s = v*t_interval;
			if (s < interval)
			{
				for (int i = 1; i <= N; i++)
					point_construct((p - 1)->x + s*i*cos(theta), (p - 1)->y + s*i*sin(theta), (p - 1)->t + t_interval*i);
			}
			else
			{
				int N_d = (int)std::floor(s / interval);
				for (int i = 1; i <= N; i++)
					for (int j = 1; j <= N_d; j++)
						point_construct((p - 1)->x + (s*i + interval*j)*cos(theta), (p - 1)->y + (s*i + interval*j)*sin(theta), (p - 1)->t + t_interval*i);
			}
			point_construct(*p);
		}
	}
}

void Environment::time_EnvironMap::line_construct(const position<double> &start, const position<double> &end)
{
	double theta = atan2(end.y - start.y, end.x - start.x);
	double dis = sqrt(pow(start.x - end.x, 2) + pow(start.y - end.y, 2));
	for (double i = 0.; i < dis; i += interval)
		point_construct(start.x + i*cos(theta), start.y + i*sin(theta));
}

void Environment::time_EnvironMap::line_construct(const time_position<double> &start, const time_position<double> &end)
{
	double theta = atan2(end.y - start.y, end.x - start.x);
	double dis = sqrt(pow(start.x - end.x, 2) + pow(start.y - end.y, 2));
	double dt = end.t - start.t;
	double v = dis / dt;
	int N = (int)std::floor(dt / t_interval);
	double s = v*t_interval;
	if (s < interval)
	{
		for (int i = 0; i <= N; i++)
			point_construct(start.x + s*i*cos(theta), start.y + s*i*sin(theta), start.t + t_interval*i);
	}
	else
	{
		int N_d = (int)std::floor(s / interval);
		for (int i = 0; i <= N; i++)
			for (int j = 0; j <= N_d; j++)
				point_construct(start.x + (s*i + interval*j)*cos(theta), start.y + (s*i + interval*j)*sin(theta), start.t + t_interval*i);
	}
}

void Environment::time_EnvironMap::guard_construct(const std::vector<position<double>> &points, const double &width)
{
	for (double i = -width / 2; i < width / 2 + interval; i += interval)
	{
		std::vector<position<double>> points_width = points;
		for (auto &p : points_width)
			p.y += i;
		line_construct(points_width);
	}
}

void Environment::time_EnvironMap::guard_construct(const std::vector<time_position<double>> &points, const double &width)
{
	for (double i = -width / 2; i < width / 2 + interval; i += interval)
	{
		std::vector<time_position<double>> points_width = points;
		for (auto &p : points_width)
			p.y += i;
		line_construct(points_width);
	}
}

void Environment::time_EnvironMap::reset()
{
	for (auto i = environment.begin(); i != environment.end(); i++)
		i->second = 0;
}

void Environment::map_construct(time_EnvironMap *environmap)
{
	/*environmap->line_construct(position<double>(-5, -1.75), position<double>(100, -1.75));
	environmap->line_construct(position<double>(-5, 5.25), position<double>(100, 5.25));*/
	//environmap->line_construct(position<double>(-5, 1.75), position<double>(100, 1.75));
	//environmap->line_construct(position<double>(-5, -5.25), position<double>(100, -5.25));
	environmap->vehicle_construct(position<double>(12, 3.5), position<double>(4.8, 1.4), 0, -6, 1.);
}