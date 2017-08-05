#include "Collision_check.h"

Environment::time_EnvironMap dynamic_environmap = Environment::time_EnvironMap(position<double>(XMIN, YMIN), position<double>(XMAX, YMAX), TMAX);

/******************************************静态碰撞检测*******************************************/
bool Collision::iscollision(const Vehicle::Node &begin_node, const Vehicle::Node &end_node)
{
	if (iscollision(end_node.x, end_node.y, end_node.theta))
		return true;
	double s_total = sqrt(pow(begin_node.x - end_node.x, 2) + pow(begin_node.y - end_node.y, 2));
	for (double s = SAFESTEP; s < s_total; s += SAFESTEP)
	{
		if (iscollision(begin_node.x + s*cos(end_node.theta), begin_node.y + s*sin(end_node.theta), end_node.theta))
			return true;
	}
	return false;
}

bool Collision::iscollision(const double &x, const double &y, const double &theta)
{
	//auto start = steady_clock::now();
	bool isunsafe = false;
	if (Vehicle::is_node_effect(x, y, theta))
	{
		double x0, y0;
		for (int i = 0; i < 3; i++)
		{
			x0 = x + (L / 6 + L*i / 3 - DL)*cos(theta);
			y0 = y + (L / 6 + L*i / 3 - DL)*sin(theta);

			if (iscollision(x0, y0))
				isunsafe = true;
		}
	}
	else
		isunsafe = true;

	//auto end = steady_clock::now();
	//cout << "车辆一个位姿： " << (end - start).count() << endl;
	return isunsafe;
}

bool Collision::iscollision(const double &x, const double &y)
{
	//坐标转换			
	double xv, yv, xr, yr;
	xr = x - map_origin.x;
	yr = y - map_origin.y;
	xv = xr*cos(map_origin.theta) + yr*sin(map_origin.theta);
	yv = -xr*sin(map_origin.theta) + yr*cos(map_origin.theta);

	int t = 0;
	double radius = sqrt(pow(L, 2) + 9 * pow(W, 2)) / 6;

	double xmin = dynamic_environmap._range()->begin()->x;
	double ymin = dynamic_environmap._range()->begin()->y;
	double interval = dynamic_environmap._interval();

	int x0 = (int)std::floor((xv - xmin) / interval);
	int y0 = (int)std::floor((yv - ymin) / interval);
	int xi = (int)std::floor((xv - radius - xmin) / interval);
	int yi = (int)std::floor((yv - radius - ymin) / interval);
	int xj = (int)std::floor((xv + radius - xmin) / interval);
	int yj = (int)std::floor((yv + radius - ymin) / interval);

	//right above
	for (int i = x0; i <= xj; i++)
		for (int j = y0; j <= yj; j++)
		{
			if (sqrt(pow(i*interval - xv, 2) + pow(j*interval - yv, 2)) <= radius)
			{
				if ((*dynamic_environmap._environment()).at({ i, j, t }) == 1)
					return true;
			}
		}
	
	//left above
	for (int i = x0; i >= xi; i--)
		for (int j = y0; j <= yj; j++)
		{
			if (sqrt(pow((i + 1)*interval - xv, 2) + pow(j*interval - yv, 2)) <= radius)
			{
				if ((*dynamic_environmap._environment()).at({ i, j, t }) == 1)
					return true;
			}
		}

	// left below
	for (int i = x0; i >= xi; i--)
		for (auto j = y0; j >= yi; j--)
		{
			if (sqrt(pow((i + 1)*interval - xv, 2) + pow((j + 1)*interval - yv, 2)) <= radius)
			{
				if ((*dynamic_environmap._environment()).at({ i, j, t }) == 1)
					return true;
			}
		}
	
	// right below
	for (int i = x0; i <= xj; i++)
		for (auto j = y0; j >= yi; j--)
		{
			if (sqrt(pow(i*interval - xv, 2) + pow((j + 1)*interval - yv, 2)) <= radius)
			{
				if ((*dynamic_environmap._environment()).at({ i, j, t }) == 1)
					return true;
			}
		}
	
	return false;
}

/******************************************动态碰撞检测*******************************************/
bool Collision::is_dynamic_collision(const Vehicle::Node &node, const double &t)
{
	return is_dynamic_collision(node.x, node.y, node.theta, t);
}

bool Collision::is_dynamic_collision(const double &x, const double &y, const double &theta, const double &t)
{
	bool isunsafe = false;
	if (Vehicle::is_node_effect(x, y, theta))
	{
		double x0, y0;
		for (int i = 0; i < 3; i++)
		{
			x0 = x + (L / 6 + L*i / 3 - DL)*cos(theta);
			y0 = y + (L / 6 + L*i / 3 - DL)*sin(theta);
			if (is_dynamic_collision(x0, y0, t))
				isunsafe = true;
		}
	}
	else
		isunsafe = true;

	//auto end = steady_clock::now();
	//cout << "车辆一个位姿： " << (end - start).count() << endl;
	return isunsafe;
}

bool Collision::is_dynamic_collision(const double &x, const double &y, const double &t)
{
	//坐标转换			
	double xv, yv, xr, yr;
	xr = x - map_origin.x;
	yr = y - map_origin.y;
	xv = xr*cos(map_origin.theta) + yr*sin(map_origin.theta);
	yv = -xr*sin(map_origin.theta) + yr*cos(map_origin.theta);

	double xmin = dynamic_environmap._range()->begin()->x;
	double ymin = dynamic_environmap._range()->begin()->y;
	double interval = dynamic_environmap._interval();
	double radius = sqrt(pow(L, 2) + 9 * pow(W, 2)) / 6;

	int x0 = (int)std::floor((xv - xmin) / interval);// *interval + xmin;
	int y0 = (int)std::floor((yv - ymin) / interval);
	int xi = (int)std::floor((xv - radius - xmin) / interval);// *interval + xmin;
	int yi = (int)std::floor((yv - radius - ymin) / interval);// *interval + ymin;
	int xj = (int)std::floor((xv + radius - xmin) / interval);// *interval + xmin;
	int yj = (int)std::floor((yv + radius - ymin) / interval);// *interval + ymin;
	if (t > dynamic_environmap._t_range())
		return false;
	int t0 = (int)std::floor(t / dynamic_environmap._time_interval());

	//bool r = false;
	//right above
	for (int i = x0; i <= xj; i++)
		for (int j = y0; j <= yj; j++)
		{
			if (sqrt(pow(i*interval + xmin - xv, 2) + pow(j*interval + ymin - yv, 2)) <= radius)
			{
				if ((*dynamic_environmap._environment()).at({ i, j, t0 }) != 0)
					return true;
			}
		}

	//left above
	for (int i = x0; i >= xi; i--)
		for (int j = y0; j <= yj; j++)
		{
			if (sqrt(pow(i*interval + xmin - xv, 2) + pow(j*interval + ymin - yv, 2)) <= radius)
			{
				if ((*dynamic_environmap._environment()).at({ i, j, t0 }) != 0)
					return true;
			}
		}

	// left below
	for (int i = x0; i >= xi; i--)
		for (auto j = y0; j >= yi; j--)
		{
			if (sqrt(pow(i*interval + xmin - xv, 2) + pow(j*interval + ymin - yv, 2)) <= radius)
			{
				if ((*dynamic_environmap._environment()).at({ i, j, t0 }) != 0)
					return true;
			}
		}

	// right below
	for (int i = x0; i <= xj; i++)
		for (auto j = y0; j >= yi; j--)
		{
			if (sqrt(pow(i*interval + xmin - xv, 2) + pow(j*interval + ymin - yv, 2)) <= radius)
			{
				if ((*dynamic_environmap._environment()).at({ i, j, t0 }) != 0)
					return true;
			}
		}

	return false;
}

bool Collision::_safe_time_point(const Vehicle::Node &node, const double &t_min, double *t_safe, const double t_max)
{
	*t_safe = t_min;
	double t_max_find = std::max(t_max, dynamic_environmap._t_range());
	for (double t = t_min + T_INTERVAL; t <= t_max_find; t += T_INTERVAL)
	{
		if (!is_dynamic_collision(node, t /*- ERROR_T*/))
		{
			*t_safe = t;
			return true;
		}
	}	
	return false; //没有safe的时间时返回初始时间，即最小的时间
}