#include "Vehicle.h"

vector<double> L_theta = Vehicle::ensure_k();

bool Vehicle::is_node_effect(const Vehicle::Node &new_node)
{
	// estimate point A
	//cout << new_node.x << " " << new_node.y << " " << new_node.theta << " " << endl;
	//cout << "estimate point A:" /*<< endl*/;
	double x = new_node.x - DL*cos(new_node.theta) - W*sin(new_node.theta) / 2;
	//cout << x << " ";
	if (x <= XMIN || x >= XMAX)
		return false;
	double y = new_node.y - DL*sin(new_node.theta) + W*cos(new_node.theta) / 2;
	//cout << y << endl;
	if (y <= YMIN || y >= YMAX)
		return false;

	//estimate point B
	//cout << "estimate point B:" /*<< endl*/;
	x += W*sin(new_node.theta);
	//cout << x << " ";
	if (x <= XMIN || x >= XMAX)
		return false;
	y -= W*cos(new_node.theta);
	//cout << y << endl;
	if (y <= YMIN || y >= YMAX)
		return false;

	// estimate point C
	//cout << "estimate point C:" /*<< endl*/;
	x += L*cos(new_node.theta);
	//cout << x << " ";
	if (x <= XMIN || x >= XMAX)
		return false;
	y += L*sin(new_node.theta);
	//cout << y << endl;
	if (y <= YMIN || y >= YMAX)
		return false;

	// estimate point D
	//cout << "estimate point D:" /*<< endl*/;
	x -= W*sin(new_node.theta);
	//cout << x << " ";
	if (x <= XMIN || x >= XMAX)
		return false;
	y += W*cos(new_node.theta);
	//cout << y << endl;
	if (y <= YMIN || y >= YMAX)
		return false;

	//cout << "effective" << endl;
	return true;
}

bool Vehicle::is_node_effect(const double &x0, const double &y0, const double &theta0)
{
	// estimate point A
	//cout << x0 << " " << y0 << " " << theta0 << " " << endl;
	//cout << "estimate point A:" /*<< endl*/;
	double x = x0 - DL*cos(theta0) - W*sin(theta0) / 2;
	//cout << x << " ";
	if (x <= XMIN || x >= XMAX)
		return false;
	double y = y0 - DL*sin(theta0) + W*cos(theta0) / 2;
	//cout << y << endl;
	if (y <= YMIN || y >= YMAX)
		return false;

	//estimate point B
	//cout << "estimate point B:" /*<< endl*/;
	x += W*sin(theta0);
	//cout << x << " ";
	if (x <= XMIN || x >= XMAX)
		return false;
	y -= W*cos(theta0);
	//cout << y << endl;
	if (y <= YMIN || y >= YMAX)
		return false;

	// estimate point C
	//cout << "estimate point C:" /*<< endl*/;
	x += L*cos(theta0);
	//cout << x << " ";
	if (x <= XMIN || x >= XMAX)
		return false;
	y += L*sin(theta0);
	//cout << y << endl;
	if (y <= YMIN || y >= YMAX)
		return false;

	// estimate point D
	//cout << "estimate point D:" /*<< endl*/;
	x -= W*sin(theta0);
	//cout << x << " ";
	if (x <= XMIN || x >= XMAX)
		return false;
	y += W*cos(theta0);
	//cout << y << endl;
	if (y <= YMIN || y >= YMAX)
		return false;

	//cout << "effective" << endl;
	return true;
}

vector<double> Vehicle::ensure_k()
{
	vector<double> L_t;
	L_t.push_back(8 * PI / 9);
	L_t.push_back(_L_min(*L_t.rbegin()));
	for (int i = 0; i < 6; i++) //double t = Vehicle::PI / 3; t <= 5 * Vehicle::PI / 6; t += 0.1*Vehicle::PI)
	{
		L_t.push_back(5 * PI / 6 - i*0.1*PI);
		L_t.push_back(_L_min(*L_t.rbegin()));
	}
	return L_t;
}