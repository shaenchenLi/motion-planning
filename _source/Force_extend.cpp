#include "Force_extend.h"

bool Trajectory::lanechange(const int &sign, const vector<double> &L_theta, const vector<double> &bound_condition, vector<double> *control)
{
	control->clear();

	double xg = bound_condition[0];
	double yg = sign*bound_condition[1];
	double thetag = sign*bound_condition[2];

	double alfa, l1, l2, l3, x2, y2, alfa_tmp, l1_tmp, l2_tmp, l3_tmp, x2_tmp, y2_tmp;
	bool result = false;

	for (auto a = L_theta.begin(); a != L_theta.end(); a += 2)
	{
		// calculate l1
		alfa_tmp = *a;
		l1_tmp = Vehicle::_L_min(alfa_tmp, ERROR_2);
		if (l1_tmp > xg)
			break;
		// calculate l2&l3
		l2_tmp = (xg*sin(thetag) - yg*cos(thetag) - l1_tmp*sin(thetag)) / (-cos(alfa_tmp)*sin(thetag) - sin(alfa_tmp)*cos(thetag));
		l3_tmp = (xg - l1_tmp + l2_tmp*cos(alfa_tmp)) / cos(thetag);

		if (std::min(l2_tmp, l1_tmp) < *(a + 1) || std::min(l2_tmp, l3_tmp) < Vehicle::_L_min((alfa_tmp + thetag) > PI ? (2.f*PI - alfa_tmp - thetag) : (alfa_tmp + thetag), 0.1))
			continue;

		x2_tmp = l1_tmp - l2_tmp*cos(alfa_tmp);
		y2_tmp = l2_tmp*sin(alfa_tmp);
		if (((y2_tmp + 0.5f*W*cos(alfa_tmp)) > yg + W / 2) || (x2_tmp + W / 2 * sin(alfa_tmp) - L_F_BA*cos(alfa_tmp)) > xg)// path can't exceed the allowed lane
			continue;

		result = true;
		alfa = alfa_tmp;
		l1 = l1_tmp;
		l2 = l2_tmp;
		l3 = l3_tmp;
		x2 = x2_tmp;
		y2 = y2_tmp;
	}
	if (result == true)
	{
		control->push_back(0); control->push_back(0);
		control->push_back(l1); control->push_back(0);
		control->push_back(x2); control->push_back(sign*y2);
		control->push_back(xg); control->push_back(sign*yg);
	}
	return result;
}

bool Trajectory::lanechange_c(const int &sign, const vector<double> &L_theta, const vector<double> &bound_condition, const vector<double> &constraints, vector<double> *control)
{
	control->clear();

	double xg = bound_condition[0];
	double yg = sign*bound_condition[1];
	double thetag = sign*bound_condition[2];
	double l = constraints[0];
	double w = constraints[1];
	
	double alfa, l1, l2, l3, x2, y2, alfa_tmp, l1_tmp, l2_tmp, l3_tmp, x2_tmp, y2_tmp;
	bool result = false;
	for (auto a = L_theta.begin(); a != L_theta.end(); a += 2)
	{
		// calculate l1
		alfa_tmp = *a;
		l1_tmp = std::min(Vehicle::_L_min(alfa_tmp, ERROR_2), (l + (w - ERROR_1) / tan(alfa_tmp) - W / 2 / sin(alfa_tmp)));
		if (l1_tmp > l - L_F_BA + ERROR_1 || l1_tmp >= xg)
			break;
		// calculate l2&l3
		l2_tmp = (xg*sin(thetag) - yg*cos(thetag) - l1_tmp*sin(thetag)) / (-cos(alfa_tmp)*sin(thetag) - sin(alfa_tmp)*cos(thetag));
		l3_tmp = (xg - l1_tmp + l2_tmp*cos(alfa_tmp)) / cos(thetag);

		if (std::min(l2_tmp, l1_tmp) < *(a + 1) || std::min(l2_tmp, l3_tmp) < Vehicle::_L_min((alfa_tmp + thetag) > PI ? (2.f*PI - alfa_tmp - thetag) : (alfa_tmp + thetag)))
			continue;

		if (l2_tmp*sin(alfa_tmp) < w)
		{
			if (w < ((l - l1_tmp - l2_tmp*cos(alfa_tmp) - W / 2 * sin(thetag))*tan(thetag) + l2_tmp*sin(alfa_tmp) + ERROR_1))
				continue;
		}

		x2_tmp = l1_tmp - l2_tmp*cos(alfa_tmp);
		y2_tmp = l2_tmp*sin(alfa_tmp);
		if (((y2_tmp + 0.5f*W*cos(alfa_tmp))>yg + W / 2) || (x2_tmp + W / 2 * sin(alfa_tmp) - L_F_BA*cos(alfa_tmp)) > xg)// path can't exceed the allowed lane
			continue;

		result = true;
		alfa = alfa_tmp;
		l1 = l1_tmp;
		l2 = l2_tmp;
		l3 = l3_tmp;
		x2 = x2_tmp;
		y2 = y2_tmp;
	}
	if (result == true)
	{
		control->push_back(0); control->push_back(0);
		control->push_back(l1); control->push_back(0);
		control->push_back(x2); control->push_back(sign*y2);
		control->push_back(xg); control->push_back(sign*yg);
	}
	return result;
}

bool Trajectory::turn(const int &sign, const vector<double> &L_theta, const vector<double> &bound_condition, vector<double> *control)
{
	control->clear();
	
	double xg = bound_condition[0];
	double yg = sign*bound_condition[1];
	double thetag = sign*bound_condition[2];

	bool result = false;

	if (thetag == PI / 4)
		return result;

	double alfa, l1, l2, l3, x2, y2, alfa_tmp, l1_tmp, l2_tmp, l3_tmp, x2_tmp, y2_tmp;
	double step = 0.1;
	double exceed_allow_x = 3;
	double exceed_allow_y = 5;
	for (auto a = L_theta.begin(); a != L_theta.end() - 4; a += 2)
	{
		// calculate l1
		alfa_tmp = *a;
		l1_tmp = Vehicle::_L_min(alfa_tmp, ERROR_2) - step;
		if (l1_tmp > xg + exceed_allow_x)
			break;
		while (result == false)
		{
			l1_tmp += step;
			if (l1_tmp > xg + exceed_allow_x)
				break;
			// calculate l2&l3
			l2_tmp = (xg*sin(thetag) - yg*cos(thetag) - l1_tmp*sin(thetag)) / (-cos(alfa_tmp)*sin(thetag) - sin(alfa_tmp)*cos(thetag));
			l3_tmp = (yg - l2_tmp*sin(alfa_tmp)) / sin(thetag);

			if (std::min(l2_tmp, l1_tmp) < *(a + 1) || std::min(l2_tmp, l3_tmp) < Vehicle::_L_min(2.f*PI - alfa_tmp - thetag))
				continue;

			x2_tmp = l1_tmp - l2_tmp*cos(alfa_tmp);
			y2_tmp = l2_tmp*sin(alfa_tmp);
			if (y2_tmp >= (yg - exceed_allow_y) || x2_tmp > (xg + exceed_allow_x))
				continue;

			result = true;
			alfa = alfa_tmp;
			l1 = l1_tmp;
			l2 = l2_tmp;
			l3 = l3_tmp;
			x2 = x2_tmp;
			y2 = y2_tmp;
		}
	}
	if (result == true)
	{
		control->push_back(0); control->push_back(0);
		control->push_back(l1); control->push_back(0);
		control->push_back(x2); control->push_back(sign*y2);
		control->push_back(xg); control->push_back(sign*yg);
	}
	return result;
}

bool Trajectory::turn_c(const vector<double> &L_theta, const vector<double> &bound_condition, const vector<double> &constraints, vector<double> *control)
{
	control->clear();

	double xg = bound_condition[0];
	double yg = bound_condition[1];
	double thetag = bound_condition[2];
	double l = constraints[0];
	double w = constraints[1];
	double r = constraints[2];

	bool result = false;

	if (thetag == PI / 4)
		return result;

	double alfa, l1, l2, l3, x2, y2, alfa_tmp, l1_tmp, l2_tmp, l3_tmp, x2_tmp, y2_tmp;
	double step = 0.1f;
	double exceed_allow_x = 3.f;
	double exceed_allow_y = 5.f;
	for (auto a = L_theta.begin(); a != L_theta.end() - 4; a += 2)
	{
		if (thetag == PI / 4)
			break;
		if (result == true)
			break;
		// calculate l1
		alfa_tmp = *a;
		l1_tmp = Vehicle::_L_min(alfa_tmp) - step;
		while (result == false)
		{
			l1_tmp += step;
			if (l1_tmp > xg + exceed_allow_x)
				break;

			l2_tmp = (xg*sin(thetag) - yg*cos(thetag) - l1_tmp*sin(thetag)) / (-cos(alfa_tmp)*sin(thetag) - sin(alfa_tmp)*cos(thetag));
			l3_tmp = (yg - l2_tmp*sin(alfa_tmp)) / sin(thetag);

			if (std::min(l2_tmp, l1_tmp) < *(a + 1) || std::min(l2_tmp, l3_tmp) < Vehicle::_L_min(2.f*PI - alfa_tmp - thetag))
				continue;

			x2_tmp = l1_tmp - l2_tmp*cos(alfa_tmp);
			y2_tmp = l2_tmp*sin(alfa_tmp);
			if (x2_tmp > (xg + exceed_allow_x) || y2_tmp >= yg - exceed_allow_y)
				continue;

			if (atan2(w, l + r - l1_tmp) > alfa_tmp)
				break;
			double beita = PI - atan2(w, l + r + L_F_BA - l1_tmp) - alfa_tmp;
			double d = sqrt(pow((l + r + L_F_BA - l1_tmp), 2) + pow(w, 2))*sin(beita);
			double x_v = l1_tmp - d / tan(beita)*cos(alfa_tmp);
			double y_v = d / tan(beita)*sin(alfa_tmp);
			if (x_v <= x2_tmp && y_v <= y2_tmp)
			{
				if (d < (r + 0.5f*W + ERROR_3))
					continue;
				if (d > r + 0.5f*W + 1.f)
					continue;
			}
			else
			{
				beita = alfa_tmp + thetag - atan2(l + r + L_F_BA - x2_tmp, w - y2_tmp) - PI;
				d = sqrt(pow((l + r + L_F_BA - x2_tmp), 2) + pow((w - y2_tmp), 2))*sin(beita);
				if (d < (r + 0.5f*W + ERROR_3))
					continue;
				if (d > r + 0.5f*W + 1.f)
					continue;
			}

			result = true;
			alfa = alfa_tmp;
			l1 = l1_tmp;
			l2 = l2_tmp;
			l3 = l3_tmp;
			x2 = x2_tmp;
			y2 = y2_tmp;
			break;
		}		
	}
	if (result == true)
	{
		control->push_back(0); control->push_back(0);
		control->push_back(l1); control->push_back(0);
		control->push_back(x2); control->push_back(y2);
		control->push_back(xg); control->push_back(yg);
	}
	return result;
}

bool Trajectory::U_turn(const vector<double> &L_theta, const vector<double> &bound_condition, vector<double> *control)
{
	control->clear();

	double xg = bound_condition[0];
	double yg = bound_condition[1];
	double thetag = bound_condition[2];

	double alfa, beita, l1, l2, l3, l4, alfa_tmp, beita_tmp, l1_tmp, l2_tmp, l3_tmp, l4_tmp;
	double P1[2], P2[2], P3[2], P1_tmp[2], P2_tmp[2], P3_tmp[2];
	bool result = false;

	double exceed_allow_x = 8;
	double exceed_allow_y = 1;
	double step = 0.2f;
	for (auto a = L_theta.begin(); a != L_theta.end(); a += 2)
	{
		if (result == true)
			break;
		// calculate l1
		alfa_tmp = *a;
		l1_tmp = *(a + 1) - step;
		while (result == false)
		{
			if (result == true)
				break;
			l1_tmp += step;
			if (l1_tmp > xg + exceed_allow_x)
				break;
			P1_tmp[0] = l1_tmp;
			P1_tmp[1] = 0;

			for (auto b = L_theta.begin(); b != L_theta.end(); b += 2)
			{
				if (result == true)
					break;
				beita_tmp = *b;
				l4_tmp = Vehicle::_L_min(beita_tmp) - step;
				while (1)
				{
					l4_tmp += step;
					P3_tmp[0] = xg - l4_tmp*cos(thetag);
					P3_tmp[1] = yg - l4_tmp*sin(thetag);
					if (P3_tmp[0] > xg + exceed_allow_x || P3_tmp[1] > exceed_allow_y + yg)
						break;

					l3_tmp = (xg*sin(alfa_tmp) + yg*cos(alfa_tmp) - l4_tmp*(cos(thetag)*sin(alfa_tmp) + sin(thetag)*cos(alfa_tmp) - l1_tmp*sin(alfa_tmp)));
					l3_tmp = l3_tmp / (cos(beita_tmp + thetag)*sin(alfa_tmp) + sin(beita_tmp + thetag)*cos(alfa_tmp));
					l2_tmp = (yg - l4_tmp*sin(thetag) - l3_tmp*sin(beita_tmp + thetag)) / sin(alfa_tmp);
					
					P2_tmp[0] = l1_tmp - l2_tmp*cos(alfa_tmp);
					P2_tmp[1] = l2_tmp*sin(alfa_tmp);
					if (P2_tmp[0] > xg + exceed_allow_x || P2_tmp[1] > exceed_allow_y + yg)
						continue;

					if (std::min(l1_tmp, l2_tmp) < Vehicle::_L_min(alfa_tmp))
						continue;

					if (std::min(l3_tmp, l4_tmp) < Vehicle::_L_min(beita_tmp))
						continue;

					double gama = (P1_tmp[0] - P2_tmp[0])*(P3_tmp[0] - P2_tmp[0]) + (P1_tmp[1] - P2_tmp[1])*(P3_tmp[1] - P2_tmp[1]);
					gama = gama / ((sqrt(pow((P1_tmp[0] - P2_tmp[0]), 2) + pow((P1_tmp[1] - P2_tmp[1]), 2)))*(sqrt(pow((P3_tmp[0] - P2_tmp[0]), 2) + pow((P3_tmp[1] - P2_tmp[1]), 2))));
					gama = acos(gama);
					if (std::min(l2_tmp, l3_tmp) < Vehicle::_L_min(gama))
						continue;

					result = true;
					alfa = alfa_tmp;
					beita = beita_tmp;
					l1 = l1_tmp;
					l2 = l2_tmp;
					l3 = l3_tmp;
					l4 = l4_tmp;
					P1[0] = P1_tmp[0]; P1[1] = P1_tmp[1];
					P2[0] = P2_tmp[0]; P2[1] = P2_tmp[1];
					P3[0] = P3_tmp[0]; P3[1] = P3_tmp[1];
					break;
				}
			}
		}
	}
	if (result == true)
	{
		control->push_back(0); control->push_back(0);
		control->push_back(P1[0]); control->push_back(P1[1]);
		control->push_back(P2[0]); control->push_back(P2[1]);
		control->push_back(P3[0]); control->push_back(P3[1]);
		control->push_back(xg); control->push_back(yg);
	}
	return result;
}

bool Trajectory::U_turn_c(const vector<double> &L_theta, const vector<double> &bound_condition, const vector<double> &constraints, vector<double> *control)
{
	control->clear();

	bool result = false;

	double xg = bound_condition[0];
	double yg = bound_condition[1];
	double thetag = bound_condition[2];
	double l = constraints[0];
	double w = constraints[1];

	double alfa, beita, l1, l2, l3, l4, alfa_tmp, beita_tmp, l1_tmp, l2_tmp, l3_tmp, l4_tmp;
	double P1[2], P2[2], P3[2], P1_tmp[2], P2_tmp[2], P3_tmp[2];

	double exceed_allow_x = 8;
	double exceed_allow_y = 1;
	double step = 0.2f;
	for (auto a = L_theta.begin(); a != L_theta.end(); a += 2)
	{
		if (result == true)
			break;
		// calculate l1
		alfa_tmp = *a;
		l1_tmp = *(a + 1) - step;
		while (result == false)
		{
			if (result == true)
				break;
			l1_tmp += step;
			if (l1_tmp > xg + exceed_allow_x)
				break;
			P1_tmp[0] = l1_tmp;
			P1_tmp[1] = 0;

			for (auto b = L_theta.begin(); b != L_theta.end(); b += 2)
			{
				if (result == true)
					break;
				beita_tmp = *b;
				l4_tmp = Vehicle::_L_min(beita_tmp) - step;
				while (1)
				{
					l4_tmp += step;
					P3_tmp[0] = xg - l4_tmp*cos(thetag);
					P3_tmp[1] = yg - l4_tmp*sin(thetag);
					if (P3_tmp[0] > xg + exceed_allow_x || P3_tmp[1] > exceed_allow_y + yg)
						break;

					l3_tmp = (xg*sin(alfa_tmp) + yg*cos(alfa_tmp) - l4_tmp*(cos(thetag)*sin(alfa_tmp) + sin(thetag)*cos(alfa_tmp) - l1_tmp*sin(alfa_tmp)));
					l3_tmp = l3_tmp / (cos(beita_tmp + thetag)*sin(alfa_tmp) + sin(beita_tmp + thetag)*cos(alfa_tmp));
					l2_tmp = (yg - l4_tmp*sin(thetag) - l3_tmp*sin(beita_tmp + thetag)) / sin(alfa_tmp);
					
					P2_tmp[0] = l1_tmp - l2_tmp*cos(alfa_tmp);
					P2_tmp[1] = l2_tmp*sin(alfa_tmp);
					if (P2_tmp[0] > xg + exceed_allow_x || P2_tmp[1] > exceed_allow_y + yg)
						continue;

					if (std::min(l1_tmp, l2_tmp) < Vehicle::_L_min(alfa_tmp))
						continue;

					if (std::min(l3_tmp, l4_tmp) < Vehicle::_L_min(beita_tmp))
						continue;

					double gama = (P1_tmp[0] - P2_tmp[0])*(P3_tmp[0] - P2_tmp[0]) + (P1_tmp[1] - P2_tmp[1])*(P3_tmp[1] - P2_tmp[1]);
					gama = gama / ((sqrt(pow((P1_tmp[0] - P2_tmp[0]), 2) + pow((P1_tmp[1] - P2_tmp[1]), 2)))*(sqrt(pow((P3_tmp[0] - P2_tmp[0]), 2) + pow((P3_tmp[1] - P2_tmp[1]), 2))));
					gama = acos(gama);
					if (std::min(l2_tmp, l3_tmp) < Vehicle::_L_min(gama))
						continue;

					if (w < P2_tmp[2])
					{
						if ((w*tan(alfa_tmp - PI / 2) + l1_tmp) < (l + w*0.5f / cos(alfa_tmp - PI / 2)))
							continue;
					}
					else if (w < P3_tmp[2])
					{
						if ((P2_tmp[0] - (w - P2_tmp[1]) / tan(gama + alfa_tmp - PI)) < (l + w*0.5f / sin(gama + alfa_tmp - PI)))
							continue;
					}
					else if (w<yg&&w>P3_tmp[2])
					{
						if ((xg - (yg - w)*tan(thetag)) < (l - w*0.5f / sin(thetag)))
							continue;
					}

					result = true;
					alfa = alfa_tmp;
					beita = beita_tmp;
					l1 = l1_tmp;
					l2 = l2_tmp;
					l3 = l3_tmp;
					l4 = l4_tmp;
					P1[0] = P1_tmp[0]; P1[1] = P1_tmp[1];
					P2[0] = P2_tmp[0]; P2[1] = P2_tmp[1];
					P3[0] = P3_tmp[0]; P3[1] = P3_tmp[1];
					break;
				}
			}
		}
	}
	if (result == true)
	{
		control->push_back(0); control->push_back(0);
		control->push_back(P1[0]); control->push_back(P1[1]);
		control->push_back(P2[0]); control->push_back(P2[1]);
		control->push_back(P3[0]); control->push_back(P3[1]);
		control->push_back(xg); control->push_back(yg);
	}
	return result;
}

int Trajectory::is_force_extend_safe(const Vehicle::Node &startnode, const double &step, const vector<double> &control, Collision::collision *collimap, double *le, vector<Vehicle::Node> *path)
{
	ts::BSpline bspline = ts::BSpline(3, 2, control.size() - 1, TS_CLAMPED);
	vector<ts::rational> ctrl_points = bspline.ctrlp();
	ctrl_points[0] = control[0] + startnode.x;				ctrl_points[1] = control[1] + startnode.y;
	for (int i = 2; i < ctrl_points.size(); i += 4)
	{
		ctrl_points[i] = 0.5f*(control[1 + i / 2] + control[i / 2 - 1]) + startnode.x;
		ctrl_points[i + 1] = 0.5f*(control[2 + i / 2] + control[i / 2]) + startnode.y;
		ctrl_points[i + 2] = control[1 + i / 2] + startnode.x;
		ctrl_points[i + 3] = control[2 + i / 2] + startnode.y;
	}
	bspline.setCtrlp(ctrl_points);  //在原控制点中插入中点生成所需要的B样条曲线

	int issafe = 2;
	double du = 0.001f;
	double u = 0.f;
	double s = 0.f;
	int count = 0;
	*le = 0;
	double le_tmp = 0;
	int time = (int)std::floor(step / SAFESTEP);
	vector<double> old_tmp{ startnode.x, startnode.y };
	Point2D diff_1, diff_2, old_diff(0, 0);
	for (; u < 1.f; u += du)
	{
		auto result = bspline.evaluate(u).result();
		diff_1.reset(result[0] - old_tmp[0], result[1] - old_tmp[1]);
		diff_2.reset(diff_1.x - old_diff.x, diff_1.y - old_diff.y);
		double theta = atan2(diff_1.y, diff_1.x) + startnode.theta;
		s += sqrt(pow(result[0] - old_tmp[0], 2.f) + pow(result[1] - old_tmp[1], 2.f));
		old_tmp[0] = result[0];          old_tmp[1] = result[1];
		old_diff = diff_1;
		if (s < SAFESTEP - ERROR_1)
			continue;
		if (collimap->iscollision(result[0], result[1], theta))
		{
			issafe = 1;
			break;
		}
		double k = (diff_1.x*diff_2.y - diff_2.x*diff_1.y) / pow(sqrt(pow(diff_1.x, 2.f) + pow(diff_2.y, 2.f)), 1.5);

		le_tmp += s;
		s = 0;
		count++;
		if (count == time)
		{
			*le = le_tmp;
			count = 0;
		}
	}


	//根据u的最终值判断最后一个正确的控制点，插入时不用插入第一个控制点
	int end_safe = (int)std::floor(u*(ctrl_points.size() / 2 - 3)) + 3;
	Vehicle::Node old_node(startnode);
	for (int i = 2; i <= end_safe - 1; i += 2)
	{
		path->emplace_back(Vehicle::Node(ctrl_points[2 * i], ctrl_points[2 * i + 1], atan2(ctrl_points[2 * i + 1] - old_node.y, ctrl_points[2 * i] - old_node.x) + startnode.theta));
		old_node = *path->rbegin();
	}

	//对于issafe为1的情况有两种特殊情况，一种是是最后一个新插入的中点，那么需要将最后一个控制点弹出；一种是没有插入，那么将issafe重设为0（第二种要在第一种之后判断）
	if (issafe == 1)
	{
		if (u >= 1 - 1 / (ctrl_points.size() / 2 - 3))
			path->pop_back();
		if (path->size() == 0)
			issafe = 0;
	}

	return issafe;
}