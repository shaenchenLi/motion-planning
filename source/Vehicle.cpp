#include "Vehicle.h"

Vehicle::Node map_origin = Vehicle::Node();

vector<double> L_theta = Vehicle::ensure_k();

bool Vehicle::is_node_effect(const double &x0, const double &y0, const double &theta0)
{
	double xv, yv, thetav, xr, yr;
	xr = x0 - map_origin.x;
	yr = y0 - map_origin.y;
	xv = xr*cos(map_origin.theta) + yr*sin(map_origin.theta);
	yv = -xr*sin(map_origin.theta) + yr*cos(map_origin.theta);
	thetav = theta0 - map_origin.theta;

	// estimate point A
	//cout << x0 << " " << y0 << " " << theta0 << " " << endl;
	//cout << "estimate point A:" /*<< endl*/;
	double x = xv - DL*cos(thetav) - W*sin(thetav) / 2;
	//cout << x << " ";
	if (x <= XMIN || x >= XMAX)
		return false;
	double y = yv - DL*sin(thetav) + W*cos(thetav) / 2;
	//cout << y << endl;
	if (y <= YMIN || y >= YMAX)
		return false;

	//estimate point B
	//cout << "estimate point B:" /*<< endl*/;
	x += W*sin(thetav);
	//cout << x << " ";
	if (x <= XMIN || x >= XMAX)
		return false;
	y -= W*cos(thetav);
	//cout << y << endl;
	if (y <= YMIN || y >= YMAX)
		return false;

	// estimate point C
	//cout << "estimate point C:" /*<< endl*/;
	x += L*cos(thetav);
	//cout << x << " ";
	if (x <= XMIN || x >= XMAX)
		return false;
	y += L*sin(thetav);
	//cout << y << endl;
	if (y <= YMIN || y >= YMAX)
		return false;

	// estimate point D
	//cout << "estimate point D:" /*<< endl*/;
	x -= W*sin(thetav);
	//cout << x << " ";
	if (x <= XMIN || x >= XMAX)
		return false;
	y += W*cos(thetav);
	//cout << y << endl;
	if (y <= YMIN || y >= YMAX)
		return false;

	//cout << "effective" << endl;
	return true;
}

bool Vehicle::is_node_effect(const Vehicle::Node &new_node)
{
	return is_node_effect(new_node.x, new_node.y, new_node.theta);
}

vector<double> Vehicle::ensure_k()
{
	vector<double> L_t;

	for (int i = 0; i < 8; i++) 
	{
		L_t.push_back(17 * PI / 18 - i*PI / 18);
		L_t.push_back(_L_min(*L_t.rbegin()));
	}

	for (int i = 0; i < 8; i++)
	{
		L_t.push_back(19 * PI / 18 + i*PI / 18);
		L_t.push_back(_L_min(*L_t.rbegin()));
	}

	return L_t;
}

void Vehicle::orient_trans_global_vehicle(const Vehicle::Node &xi, Vehicle::Node *xg)
{
	xg->theta = xg->theta - xi.theta;

	double xg_r = xg->x - xi.x;
	double yg_r = xg->y - xi.y;
	xg->x = xg_r*cos(xi.theta) + yg_r*sin(xi.theta);
	xg->y = -xg_r*sin(xi.theta) + yg_r*cos(xi.theta);
}

void Vehicle::orient_trans_global_vehicle(const Vehicle::Node &xi, vector<double> *control)
{
	double x0 = xi.x, y0 = xi.y, theta0 = xi.theta;
	double xg_r, yg_r;

	for (auto i = control->begin(); i != control->end(); i += 2)
	{
		xg_r = *i - x0;
		yg_r = *(i + 1) - y0;

		*i = xg_r*cos(theta0) + yg_r*sin(theta0);
		*(i + 1) = -xg_r*sin(theta0) + yg_r*cos(theta0);
	}
}

void Vehicle::orient_trans_vehicle_global(const Vehicle::Node &xi, Vehicle::Node *xg)
{
	double x0 = xi.x, y0 = xi.y, theta0 = xi.theta;
	double xg_r = xg->x - x0; 
	double yg_r = xg->y - y0;

	xg->theta = xg->theta + theta0;
	xg->x = xg_r*cos(theta0) - yg_r*sin(theta0) + x0;
	xg->y = xg_r*sin(theta0) + yg_r*cos(theta0) + y0;
}

void Vehicle::orient_trans_vehicle_global(const Vehicle::Node &xi, vector<Vehicle::Node> *xg)
{
	double x0 = xi.x, y0 = xi.y, theta0 = xi.theta;
	double xg_r, yg_r;

	for (auto &i : *xg)
	{
		xg_r = i.x;
		yg_r = i.y;

		i.theta = i.theta + theta0;
		i.x = xg_r*cos(theta0) - yg_r*sin(theta0) + x0;
		i.y = xg_r*sin(theta0) + yg_r*cos(theta0) + y0;
	}
}

void Vehicle::orient_trans_vehicle_global(const Vehicle::Node &xi, vector<double> *control)
{
	double x0 = xi.x, y0 = xi.y, theta0 = xi.theta;
	double xg_r, yg_r;

	for (auto i = control->begin(); i != control->end(); i += 2)
	{
		xg_r = *i;
		yg_r = *(i + 1);

		*i = xg_r*cos(theta0) - yg_r*sin(theta0) + x0;
		*(i + 1) = xg_r*sin(theta0) + yg_r*cos(theta0) + y0;
	}
}