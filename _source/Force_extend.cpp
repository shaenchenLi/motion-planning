#include "Force_extend.h"

bool Trajectory::lanechange(const int &sign, const vector<float> &L_theta, const vector<float> &bound_condition, vector<float> *control)
{
	control->clear();

	float xg = bound_condition[0];
	float yg = sign*bound_condition[1];
	float thetag = sign*bound_condition[2];

	float alfa, l1, l2, l3, x2, y2, alfa_tmp, l1_tmp, l2_tmp, l3_tmp, x2_tmp, y2_tmp;
	bool result = false;

	for (auto a = L_theta.begin(); a != L_theta.end(); a += 2)
	{
		// calculate l1
		alfa_tmp = *a;
		l1_tmp = Vehicle::_L_min(alfa_tmp, ERROR_2);
		if (l1_tmp > xg)
			break;
		// calculate l2&l3
		l2_tmp = (xg*sinf(thetag) - yg*cosf(thetag) - l1_tmp*sinf(thetag)) / (-cosf(alfa_tmp)*sinf(thetag) - sinf(alfa_tmp)*cosf(thetag));
		l3_tmp = (xg - l1_tmp + l2_tmp*cosf(alfa_tmp)) / cosf(thetag);

		if (std::min(l2_tmp, l1_tmp) < *(a + 1) || std::min(l2_tmp, l3_tmp) < Vehicle::_L_min((alfa_tmp + thetag) > PI ? (2.f*PI - alfa_tmp - thetag) : (alfa_tmp + thetag), 0.1))
			continue;

		x2_tmp = l1_tmp - l2_tmp*cosf(alfa_tmp);
		y2_tmp = l2_tmp*sinf(alfa_tmp);
		if (((y2_tmp + 0.5f*W*cosf(alfa_tmp)) > yg + W / 2) || (x2_tmp + W / 2 * sinf(alfa_tmp) - L_F_BA*cosf(alfa_tmp)) > xg)// path can't exceed the allowed lane
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

bool Trajectory::lanechange_c(const int &sign, const vector<float> &L_theta, const vector<float> &bound_condition, const vector<float> &constraints, vector<float> *control)
{
	control->clear();

	float xg = bound_condition[0];
	float yg = sign*bound_condition[1];
	float thetag = sign*bound_condition[2];
	float l = constraints[0];
	float w = constraints[1];
	
	float alfa, l1, l2, l3, x2, y2, alfa_tmp, l1_tmp, l2_tmp, l3_tmp, x2_tmp, y2_tmp;
	bool result = false;
	for (auto a = L_theta.begin(); a != L_theta.end(); a += 2)
	{
		// calculate l1
		alfa_tmp = *a;
		l1_tmp = std::min(Vehicle::_L_min(alfa_tmp, ERROR_2), (l + (w - ERROR_1) / tanf(alfa_tmp) - W / 2 / sinf(alfa_tmp)));
		if (l1_tmp > l - L_F_BA + ERROR_1 || l1_tmp >= xg)
			break;
		// calculate l2&l3
		l2_tmp = (xg*sinf(thetag) - yg*cosf(thetag) - l1_tmp*sinf(thetag)) / (-cosf(alfa_tmp)*sinf(thetag) - sinf(alfa_tmp)*cosf(thetag));
		l3_tmp = (xg - l1_tmp + l2_tmp*cosf(alfa_tmp)) / cosf(thetag);

		if (std::min(l2_tmp, l1_tmp) < *(a + 1) || std::min(l2_tmp, l3_tmp) < Vehicle::_L_min((alfa_tmp + thetag) > PI ? (2.f*PI - alfa_tmp - thetag) : (alfa_tmp + thetag)))
			continue;

		if (l2_tmp*sinf(alfa_tmp) < w)
		{
			if (w < ((l - l1_tmp - l2_tmp*cosf(alfa_tmp) - W / 2 * sinf(thetag))*tanf(thetag) + l2_tmp*sinf(alfa_tmp) + ERROR_1))
				continue;
		}

		x2_tmp = l1_tmp - l2_tmp*cosf(alfa_tmp);
		y2_tmp = l2_tmp*sinf(alfa_tmp);
		if (((y2_tmp + 0.5f*W*cosf(alfa_tmp))>yg + W / 2) || (x2_tmp + W / 2 * sinf(alfa_tmp) - L_F_BA*cosf(alfa_tmp)) > xg)// path can't exceed the allowed lane
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

bool Trajectory::turn(const int &sign, const vector<float> &L_theta, const vector<float> &bound_condition, vector<float> *control)
{
	control->clear();
	
	float xg = bound_condition[0];
	float yg = sign*bound_condition[1];
	float thetag = sign*bound_condition[2];

	bool result = false;

	if (thetag == PI / 4)
		return result;

	float alfa, l1, l2, l3, x2, y2, alfa_tmp, l1_tmp, l2_tmp, l3_tmp, x2_tmp, y2_tmp;
	float step = 0.1;
	float exceed_allow_x = 3;
	float exceed_allow_y = 5;
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
			l2_tmp = (xg*sinf(thetag) - yg*cosf(thetag) - l1_tmp*sinf(thetag)) / (-cosf(alfa_tmp)*sinf(thetag) - sinf(alfa_tmp)*cosf(thetag));
			l3_tmp = (yg - l2_tmp*sinf(alfa_tmp)) / sinf(thetag);

			if (std::min(l2_tmp, l1_tmp) < *(a + 1) || std::min(l2_tmp, l3_tmp) < Vehicle::_L_min(2.f*PI - alfa_tmp - thetag))
				continue;

			x2_tmp = l1_tmp - l2_tmp*cosf(alfa_tmp);
			y2_tmp = l2_tmp*sinf(alfa_tmp);
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

bool Trajectory::turn_c(const vector<float> &L_theta, const vector<float> &bound_condition, const vector<float> &constraints, vector<float> *control)
{
	control->clear();

	float xg = bound_condition[0];
	float yg = bound_condition[1];
	float thetag = bound_condition[2];
	float l = constraints[0];
	float w = constraints[1];
	float r = constraints[2];

	bool result = false;

	if (thetag == PI / 4)
		return result;

	float alfa, l1, l2, l3, x2, y2, alfa_tmp, l1_tmp, l2_tmp, l3_tmp, x2_tmp, y2_tmp;
	float step = 0.1f;
	float exceed_allow_x = 3.f;
	float exceed_allow_y = 5.f;
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

			l2_tmp = (xg*sinf(thetag) - yg*cosf(thetag) - l1_tmp*sinf(thetag)) / (-cosf(alfa_tmp)*sinf(thetag) - sinf(alfa_tmp)*cosf(thetag));
			l3_tmp = (yg - l2_tmp*sinf(alfa_tmp)) / sinf(thetag);

			if (std::min(l2_tmp, l1_tmp) < *(a + 1) || std::min(l2_tmp, l3_tmp) < Vehicle::_L_min(2.f*PI - alfa_tmp - thetag))
				continue;

			x2_tmp = l1_tmp - l2_tmp*cosf(alfa_tmp);
			y2_tmp = l2_tmp*sinf(alfa_tmp);
			if (x2_tmp > (xg + exceed_allow_x) || y2_tmp >= yg - exceed_allow_y)
				continue;

			if (atan2f(w, l + r - l1_tmp) > alfa_tmp)
				break;
			float beita = PI - atan2f(w, l + r + L_F_BA - l1_tmp) - alfa_tmp;
			float d = sqrtf(powf((l + r + L_F_BA - l1_tmp), 2) + powf(w, 2))*sinf(beita);
			float x_v = l1_tmp - d / tanf(beita)*cosf(alfa_tmp);
			float y_v = d / tanf(beita)*sinf(alfa_tmp);
			if (x_v <= x2_tmp && y_v <= y2_tmp)
			{
				if (d < (r + 0.5f*W + ERROR_3))
					continue;
				if (d > r + 0.5f*W + 1.f)
					continue;
			}
			else
			{
				beita = alfa_tmp + thetag - atan2f(l + r + L_F_BA - x2_tmp, w - y2_tmp) - PI;
				d = sqrtf(powf((l + r + L_F_BA - x2_tmp), 2) + powf((w - y2_tmp), 2))*sinf(beita);
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

bool Trajectory::U_turn(const vector<float> &L_theta, const vector<float> &bound_condition, vector<float> *control)
{
	control->clear();

	float xg = bound_condition[0];
	float yg = bound_condition[1];
	float thetag = bound_condition[2];

	float alfa, beita, l1, l2, l3, l4, alfa_tmp, beita_tmp, l1_tmp, l2_tmp, l3_tmp, l4_tmp;
	float P1[2], P2[2], P3[2], P1_tmp[2], P2_tmp[2], P3_tmp[2];
	bool result = false;

	float exceed_allow_x = 8;
	float exceed_allow_y = 1;
	float step = 0.2f;
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
					P3_tmp[0] = xg - l4_tmp*cosf(thetag);
					P3_tmp[1] = yg - l4_tmp*sinf(thetag);
					if (P3_tmp[0] > xg + exceed_allow_x || P3_tmp[1] > exceed_allow_y + yg)
						break;

					l3_tmp = (xg*sinf(alfa_tmp) + yg*cosf(alfa_tmp) - l4_tmp*(cosf(thetag)*sinf(alfa_tmp) + sinf(thetag)*cosf(alfa_tmp) - l1_tmp*sinf(alfa_tmp)));
					l3_tmp = l3_tmp / (cosf(beita_tmp + thetag)*sinf(alfa_tmp) + sinf(beita_tmp + thetag)*cosf(alfa_tmp));
					l2_tmp = (yg - l4_tmp*sinf(thetag) - l3_tmp*sinf(beita_tmp + thetag)) / sinf(alfa_tmp);
					
					P2_tmp[0] = l1_tmp - l2_tmp*cosf(alfa_tmp);
					P2_tmp[1] = l2_tmp*sinf(alfa_tmp);
					if (P2_tmp[0] > xg + exceed_allow_x || P2_tmp[1] > exceed_allow_y + yg)
						continue;

					if (std::min(l1_tmp, l2_tmp) < Vehicle::_L_min(alfa_tmp))
						continue;

					if (std::min(l3_tmp, l4_tmp) < Vehicle::_L_min(beita_tmp))
						continue;

					float gama = (P1_tmp[0] - P2_tmp[0])*(P3_tmp[0] - P2_tmp[0]) + (P1_tmp[1] - P2_tmp[1])*(P3_tmp[1] - P2_tmp[1]);
					gama = gama / ((sqrtf(powf((P1_tmp[0] - P2_tmp[0]), 2) + powf((P1_tmp[1] - P2_tmp[1]), 2)))*(sqrtf(powf((P3_tmp[0] - P2_tmp[0]), 2) + powf((P3_tmp[1] - P2_tmp[1]), 2))));
					gama = acosf(gama);
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

bool Trajectory::U_turn_c(const vector<float> &L_theta, const vector<float> &bound_condition, const vector<float> &constraints, vector<float> *control)
{
	control->clear();

	bool result = false;

	float xg = bound_condition[0];
	float yg = bound_condition[1];
	float thetag = bound_condition[2];
	float l = constraints[0];
	float w = constraints[1];

	float alfa, beita, l1, l2, l3, l4, alfa_tmp, beita_tmp, l1_tmp, l2_tmp, l3_tmp, l4_tmp;
	float P1[2], P2[2], P3[2], P1_tmp[2], P2_tmp[2], P3_tmp[2];

	float exceed_allow_x = 8;
	float exceed_allow_y = 1;
	float step = 0.2f;
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
					P3_tmp[0] = xg - l4_tmp*cosf(thetag);
					P3_tmp[1] = yg - l4_tmp*sinf(thetag);
					if (P3_tmp[0] > xg + exceed_allow_x || P3_tmp[1] > exceed_allow_y + yg)
						break;

					l3_tmp = (xg*sinf(alfa_tmp) + yg*cosf(alfa_tmp) - l4_tmp*(cosf(thetag)*sinf(alfa_tmp) + sinf(thetag)*cosf(alfa_tmp) - l1_tmp*sinf(alfa_tmp)));
					l3_tmp = l3_tmp / (cosf(beita_tmp + thetag)*sinf(alfa_tmp) + sinf(beita_tmp + thetag)*cosf(alfa_tmp));
					l2_tmp = (yg - l4_tmp*sinf(thetag) - l3_tmp*sinf(beita_tmp + thetag)) / sinf(alfa_tmp);
					
					P2_tmp[0] = l1_tmp - l2_tmp*cosf(alfa_tmp);
					P2_tmp[1] = l2_tmp*sinf(alfa_tmp);
					if (P2_tmp[0] > xg + exceed_allow_x || P2_tmp[1] > exceed_allow_y + yg)
						continue;

					if (std::min(l1_tmp, l2_tmp) < Vehicle::_L_min(alfa_tmp))
						continue;

					if (std::min(l3_tmp, l4_tmp) < Vehicle::_L_min(beita_tmp))
						continue;

					float gama = (P1_tmp[0] - P2_tmp[0])*(P3_tmp[0] - P2_tmp[0]) + (P1_tmp[1] - P2_tmp[1])*(P3_tmp[1] - P2_tmp[1]);
					gama = gama / ((sqrtf(powf((P1_tmp[0] - P2_tmp[0]), 2) + powf((P1_tmp[1] - P2_tmp[1]), 2)))*(sqrtf(powf((P3_tmp[0] - P2_tmp[0]), 2) + powf((P3_tmp[1] - P2_tmp[1]), 2))));
					gama = acosf(gama);
					if (std::min(l2_tmp, l3_tmp) < Vehicle::_L_min(gama))
						continue;

					if (w < P2_tmp[2])
					{
						if ((w*tanf(alfa_tmp - PI / 2) + l1_tmp) < (l + w*0.5f / cosf(alfa_tmp - PI / 2)))
							continue;
					}
					else if (w < P3_tmp[2])
					{
						if ((P2_tmp[0] - (w - P2_tmp[1]) / tanf(gama + alfa_tmp - PI)) < (l + w*0.5f / sinf(gama + alfa_tmp - PI)))
							continue;
					}
					else if (w<yg&&w>P3_tmp[2])
					{
						if ((xg - (yg - w)*tanf(thetag)) < (l - w*0.5f / sinf(thetag)))
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

int Trajectory::is_force_extend_safe(const Vehicle::Node &startnode, const float &step, const vector<float> &control, Collision::collision *collimap, float *le, vector<Vehicle::Node> *path)
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
	float du = 0.001f;
	float u = 0.f;
	float s = 0.f;
	int count = 0;
	*le = 0;
	float le_tmp = 0;
	int time = (int)std::floor(step / SAFESTEP);
	vector<float> old_tmp{ startnode.x, startnode.y };
	Point2D diff_1, diff_2, old_diff(0, 0);
	for (; u < 1.f; u += du)
	{
		auto result = bspline.evaluate(u).result();
		diff_1.reset(result[0] - old_tmp[0], result[1] - old_tmp[1]);
		diff_2.reset(diff_1.x - old_diff.x, diff_1.y - old_diff.y);
		float theta = atan2f(diff_1.y, diff_1.x) + startnode.theta;
		s += sqrtf(powf(result[0] - old_tmp[0], 2.f) + powf(result[1] - old_tmp[1], 2.f));
		old_tmp[0] = result[0];          old_tmp[1] = result[1];
		old_diff = diff_1;
		if (s < SAFESTEP - ERROR_1)
			continue;
		if (collimap->iscollision(result[0], result[1], theta))
		{
			issafe = 1;
			break;
		}
		float k = (diff_1.x*diff_2.y - diff_2.x*diff_1.y) / powf(sqrtf(powf(diff_1.x, 2.f) + powf(diff_2.y, 2.f)), 1.5);

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
		path->emplace_back(Vehicle::Node(ctrl_points[2 * i], ctrl_points[2 * i + 1], atan2f(ctrl_points[2 * i + 1] - old_node.y, ctrl_points[2 * i] - old_node.x) + startnode.theta));
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