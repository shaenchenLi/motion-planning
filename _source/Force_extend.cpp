#include "Force_extend.h"

bool Trajectory::straight(const vector<double> &bound_condition, vector<double> *control)
{
	control->push_back(0);						  control->push_back(0);
	control->push_back(0.25*bound_condition[0]);  control->push_back(bound_condition[1]);
	control->push_back(0.5*bound_condition[0]);   control->push_back(bound_condition[1]);
	control->push_back(0.75*bound_condition[0]);  control->push_back(bound_condition[1]);
	control->push_back(bound_condition[0]);		  control->push_back(bound_condition[1]);
	return true;
}

bool Trajectory::lanechange(const Vehicle::Node &start_node, const Vehicle::Node &end_node, vector<double> *control)
{
	control->clear();

	int sign_y, sign_x;
	if ((end_node.x - start_node.x) < 0)
	{
		sign_x = -1;
		if ((end_node.y - start_node.y) > 0)
			sign_y = -1;
		else
			sign_y = 1;
	}
	else
	{
		sign_x = 1;
		if ((end_node.y - start_node.y) < 0)
			sign_y = -1;
		else
			sign_y = 1;
	}

	double alfa_tmp, beita_tmp, l1_tmp, l2_tmp, l3_tmp, x1_tmp, y1_tmp, x2_tmp, y2_tmp;
	bool result = false;
	bool suboptimal = false;
	vector<Vector5d>* subopt = new vector<Vector5d>;
	vector<Vector5d>* opt = new vector<Vector5d>;

	for (auto a = L_theta.begin(); a != L_theta.end(); a += 2)
	{
		// calculate l1
		alfa_tmp = *a;
		l1_tmp = Vehicle::_L_min(alfa_tmp, 2 * ERROR_2);
		x1_tmp = start_node.x + l1_tmp*cos(start_node.theta);
		y1_tmp = start_node.y + l1_tmp*sin(start_node.theta);
		if (sign_x*(end_node.x - x1_tmp) < L_F_BA*std::abs(cos(start_node.theta)) + 0.5*W*std::abs(sin(start_node.theta)) + *(L_theta.end() - 2))
			break;
		if (sign_x*(end_node.x - x1_tmp) < L_F_BA*std::abs(cos(alfa_tmp - sign_y*start_node.theta)) + 0.5*W*std::abs(alfa_tmp - sign_y*sin(start_node.theta)) + *(L_theta.end() - 2))
			break;

		// calculate l2&l3
		l2_tmp = sign_y*((start_node.x - end_node.x)*sin(end_node.theta) - (start_node.y - end_node.y)*cos(end_node.theta) + l1_tmp*sin(end_node.theta - start_node.theta)) / sin(alfa_tmp - sign_y*start_node.theta + end_node.theta);
		if (l2_tmp < 0)
			continue;
		l3_tmp = (end_node.x - start_node.x - l1_tmp*cos(start_node.theta) + l2_tmp*cos(alfa_tmp - sign_y*start_node.theta)) / cos(end_node.theta);
		if (l3_tmp < 0)
			continue;
		
		beita_tmp = alfa_tmp - sign_y*(start_node.theta - end_node.theta);
		if (beita_tmp > PI)
			beita_tmp = 2 * PI - beita_tmp;
		if (std::min(l2_tmp, l1_tmp) < *(a + 1) || std::min(l2_tmp, l3_tmp) < Vehicle::_L_min(beita_tmp))
			continue;

		x2_tmp = end_node.x - l3_tmp*cos(end_node.theta);
		y2_tmp = end_node.y - l3_tmp*sin(end_node.theta);
		if ((sign_x*(end_node.x - x2_tmp) < (L_F_BA*std::abs(cos(alfa_tmp - sign_y*start_node.theta)) + 0.5*W*std::abs(sin(alfa_tmp - sign_y*start_node.theta)) + *(L_theta.end() - 2))) || (sign_x*(end_node.x - x2_tmp) < ((L_F_BA*std::abs(cos(end_node.theta))) + 0.5*W*std::abs(sin(end_node.theta)) + *(L_theta.end() - 2))))
			continue;
		if ((std::abs(y2_tmp + sign_y*L_F_BA*std::abs(sin(alfa_tmp - sign_y*start_node.theta)) + sign_y*0.5*W*std::abs(cos(alfa_tmp - sign_y*start_node.theta)) + 0.05) > std::abs(end_node.y + sign_y*0.5*LANE_WIDTH)) || (std::abs(y2_tmp + sign_y*L_F_BA*std::abs(sin(end_node.theta)) + sign_y*0.5*W*std::abs(cos(end_node.theta)) + 0.05) > std::abs(end_node.y + sign_y*0.5*LANE_WIDTH)))
		{
			suboptimal = true;
			Vector5d subopt_data;
			subopt_data << min({ l1_tmp, l2_tmp, l3_tmp }), x1_tmp, y1_tmp, x2_tmp, y2_tmp;
			subopt->push_back(subopt_data);
			continue;
		}

		result = true;
		Vector5d opt_data;
		opt_data << min({ l1_tmp, l2_tmp, l3_tmp }), x1_tmp, y1_tmp, x2_tmp, y2_tmp;
		opt->push_back(opt_data);
	}
	auto min_i = opt->begin();
	double min = 1000;
	if (result == true)
	{
		auto i = opt->begin();
		for (; i != opt->end(); i++)
		{
			if (min > (*i)[0])
			{
				min_i = i;
				min = (*i)[0];
			}
		}
	}
	else
	{
		if (suboptimal == true)
		{
			auto i = subopt->begin();
			for (; i != subopt->end(); i++)
			{
				if (min > (*i)[0])
				{
					min_i = i;
					min = (*i)[0];
				}
			}
			result = true;
		}
		else
			return false;
	}
	control->push_back(start_node.x); control->push_back(start_node.y);
	control->push_back((*min_i)[1]); control->push_back((*min_i)[2]);
	control->push_back((*min_i)[3]); control->push_back((*min_i)[4]);
	control->push_back(end_node.x); control->push_back(end_node.y);
	delete subopt, opt;
	return result;
}

bool Trajectory::lanechange_c(const Vehicle::Node &start_node, const Vehicle::Node &end_node, const vector<double> &constraints, vector<double> *control)
{
	control->clear();

	int sign_y;
	int sign_x;
	if ((end_node.x - start_node.x) < 0)
	{
		sign_x = -1;
		if ((end_node.y - start_node.y) > 0)
			sign_y = -1;
		else
			sign_y = 1;
	}
	else
	{
		sign_x = 1;
		if ((end_node.y - start_node.y) < 0)
			sign_y = -1;
		else
			sign_y = 1;
	}

	double alfa_tmp, beita_tmp, theta2_tmp, l1_tmp, l2_tmp, l3_tmp, x1_tmp, y1_tmp, x2_tmp, y2_tmp;
	bool result = false;
	bool suboptimal = false;
	vector<Vector5d>* subopt = new vector<Vector5d>;
	vector<Vector5d>* opt = new vector<Vector5d>;

	double l = constraints[0];
	double d = constraints[1]*sign_y;//正负表示方向，和sign一样

	for (auto a = L_theta.begin(); a != L_theta.end(); a += 2)
	{
		// calculate l1
		alfa_tmp = *a;
		l1_tmp = Vehicle::_L_min(alfa_tmp, ERROR_2);

		x1_tmp = start_node.x + l1_tmp*cos(start_node.theta);
		y1_tmp = start_node.y + l1_tmp*sin(start_node.theta);
		
		l2_tmp = sign_y*((start_node.x - end_node.x)*sin(end_node.theta) - (start_node.y - end_node.y)*cos(end_node.theta) + l1_tmp*sin(end_node.theta - start_node.theta)) / sin(alfa_tmp - sign_y*start_node.theta + end_node.theta);
		if (l2_tmp < 0)
			continue;
		l3_tmp = (end_node.x - start_node.x - l1_tmp*cos(start_node.theta) + l2_tmp*cos(alfa_tmp - sign_y*start_node.theta)) / cos(end_node.theta);
		if (l3_tmp < 0)
			continue;

		beita_tmp = alfa_tmp - sign_y*(start_node.theta - end_node.theta);
		if (beita_tmp > PI)
			beita_tmp = 2 * PI - beita_tmp;
		if (std::min(l2_tmp, l1_tmp) < *(a + 1) || std::min(l2_tmp, l3_tmp) < Vehicle::_L_min(beita_tmp))
			continue;

		x2_tmp = end_node.x - l3_tmp*cos(end_node.theta);
		y2_tmp = end_node.y - l3_tmp*sin(end_node.theta);
		if ((sign_x*(end_node.x - x2_tmp) < (L_F_BA*std::abs(cos(alfa_tmp - sign_y*start_node.theta)) + 0.5*W*std::abs(sin(alfa_tmp - sign_y*start_node.theta)) + *(L_theta.end() - 2))) || (sign_x*(end_node.x - x2_tmp) < (L_F_BA*std::abs(cos(end_node.theta)) + 0.5*W*std::abs(sin(end_node.theta)) + *(L_theta.end() - 2))))
			continue;

		theta2_tmp = alfa_tmp - sign_y*start_node.theta;

		if (std::abs(x1_tmp - start_node.x) > l + L_F_BA)
		{
			if (start_node.theta == 0.)
				break;
			else
			{
				if (std::abs(d / tan(start_node.theta) + 0.5*W * sin(start_node.theta)) >= l + L_F_BA - ERROR_3)
					continue;
				if (std::abs(d / tan(theta2_tmp) + 0.5*W * sin(theta2_tmp)) >= l + L_F_BA - ERROR_3)
					continue;
			}
		}
		else
		{
			if (std::abs(y1_tmp - start_node.y) < std::abs(d))
			{
				if (l1_tmp*cos(start_node.theta) + L_F_BA*std::abs(cos(start_node.theta)) + 0.5*W*std::abs(sin(start_node.theta)) > l + L_F_BA - ERROR_3)
					break;
				if (l1_tmp*cos(start_node.theta) + L_F_BA*std::abs(cos(theta2_tmp)) + 0.5*W*std::abs(sin(theta2_tmp)) > l + L_F_BA - ERROR_3)
					break;
			}
			if (std::abs(x2_tmp - start_node.x) > l + L_F_BA)
			{
				if (theta2_tmp == 0.)
				{
					if (std::abs(d) >= std::abs(y1_tmp - 0.5*W*sign_y) - ERROR_3)
						continue;
				}
				else if (std::abs(x1_tmp - start_node.x) + std::abs(0.5*W / sin(theta2_tmp)) + (d - l1_tmp*sin(start_node.theta)) / std::abs(tan(theta2_tmp)) >= l + L_F_BA - ERROR_3)
					continue;
			}
			else
			{
				if (end_node.theta == 0.)
				{
					if (std::abs(d) >= std::abs(y2_tmp - 0.5*W*sign_y) - ERROR_3)
						continue;
				}
				else if (std::abs(x2_tmp - start_node.x) + (d - y2_tmp + start_node.y) / std::abs(tan(end_node.theta)) + std::abs(0.5*W / sin(end_node.theta)) >= l + L_F_BA - ERROR_3)
					continue;
			}
		}

		if (((sign_y*(y2_tmp - end_node.y) + L_F_BA*std::abs(sin(alfa_tmp - sign_y*start_node.theta)) + 0.5*W*std::abs(cos(alfa_tmp - sign_y*start_node.theta)) + 0.05) > 0.5*LANE_WIDTH) || ((sign_y*(y2_tmp - end_node.y) + L_F_BA*std::abs(sin(end_node.theta)) + 0.5*W*std::abs(cos(end_node.theta)) + 0.05) > 0.5*LANE_WIDTH))
		{
			suboptimal = true;
			Vector5d subopt_data;
			subopt_data << min({ l1_tmp, l2_tmp, l3_tmp }), x1_tmp, y1_tmp, x2_tmp, y2_tmp;
			subopt->push_back(subopt_data);
			continue;
		}

		result = true;
		Vector5d opt_data;
		opt_data << min({ l1_tmp, l2_tmp, l3_tmp }), x1_tmp, y1_tmp, x2_tmp, y2_tmp;
		opt->push_back(opt_data);
	}
	auto min_i = opt->begin();
	double min = 1000;
	if (result == true)
	{
		auto i = opt->begin();
		for (; i != opt->end(); i++)
		{
			if (min > (*i)[0])
			{
				min_i = i;
				min = (*i)[0];
			}
		}
	}
	else
	{
		if (suboptimal == true)
		{
			auto i = subopt->begin();
			for (; i != subopt->end(); i++)
			{
				if (min > (*i)[0])
				{
					min_i = i;
					min = (*i)[0];
				}
			}
			result = true;
		}
		else
			return false;
	}
	control->push_back(start_node.x); control->push_back(start_node.y);
	control->push_back((*min_i)[1]); control->push_back((*min_i)[2]);
	control->push_back((*min_i)[3]); control->push_back((*min_i)[4]);
	control->push_back(end_node.x); control->push_back(end_node.y);
	delete subopt, opt;
	return result;
}

bool Trajectory::turn(const Vehicle::Node &start_node, const Vehicle::Node &end_node, vector<double> *control)
{
	control->clear();

	int sign_y;
	int sign_x;
	if ((end_node.x - start_node.x) < 0)
	{
		sign_x = -1;
		if ((end_node.y - start_node.y) > 0)
			sign_y = -1;
		else
			sign_y = 1;
	}
	else
	{
		sign_x = 1;
		if ((end_node.y - start_node.y) < 0)
			sign_y = -1;
		else
			sign_y = 1;
	}
	
	double alfa_tmp, beita_tmp, gama_tmp, l1_tmp, l2_tmp, l3_tmp, l4_tmp, x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
	double delta_subopt = 1000., delta_tmp;
	vector<Vector7d> *subopt = new vector<Vector7d>;
	vector<Vector7d> *opt = new vector<Vector7d>;
	bool result = false;
	bool suboptimal = false;
	int flag_1, flag_2; //分别为两种情况的l2和l3的计算，0表示无解，1为有解中有一个分母为0的情况，2为通用情况

	double step = 0.5;
	double exceed_allow_x = 0; 
	double exceed_allow_y = 2; //超出部分到车道中心线的距离
	for (auto a = L_theta.begin(); a != L_theta.end() - 4; a += 2)
	{
		if (result == true)
			break;
		// calculate l1
		alfa_tmp = *a;
		l1_tmp = *(a + 1) - step;

		while (result == false)
		{
			l1_tmp += step;

			// calculate l2&l3
			x1_tmp = start_node.x + l1_tmp*cos(start_node.theta);
			y1_tmp = start_node.y + l1_tmp*sin(start_node.theta);
			if (sign_x*(x1_tmp - end_node.x) > exceed_allow_x)
				break;

			for (auto b = L_theta.begin(); b != L_theta.end(); b += 2)
			{
				if (result == true)
					break;
				beita_tmp = *b;
				l4_tmp = *(b + 1) - step;

				if ((sin(end_node.theta + sign_y*beita_tmp) == 0.&&sin(alfa_tmp - sign_y*start_node.theta) == 0.) || (cos(end_node.theta + sign_y*beita_tmp) == 0.&&cos(alfa_tmp - sign_y*start_node.theta) == 0.))
				{
					if ((sin(end_node.theta - sign_y*beita_tmp) == 0.&&sin(alfa_tmp - sign_y*start_node.theta) == 0.) || (cos(end_node.theta - sign_y*beita_tmp) == 0.&&cos(alfa_tmp - sign_y*start_node.theta) == 0.))
						continue;
					else
					{
						flag_1 = 0;
						if (sin(end_node.theta - sign_y*beita_tmp) == 0.&&sin(alfa_tmp - sign_y*start_node.theta) != 0.)
							flag_2 = 1;
						else
							flag_2 = 2;
					}
				}
				else
				{
					if (sin(end_node.theta + sign_y*beita_tmp) == 0.&&sin(alfa_tmp - sign_y*start_node.theta) != 0.)
					{
						flag_1 = 1;
						if ((sin(end_node.theta - sign_y*beita_tmp) == 0.&&sin(alfa_tmp - sign_y*start_node.theta) == 0.) || (cos(end_node.theta - sign_y*beita_tmp) == 0.&&cos(alfa_tmp - sign_y*start_node.theta) == 0.))
							flag_2 = 0;
						else
						{
							if (sin(end_node.theta - sign_y*beita_tmp) == 0.&&sin(alfa_tmp - sign_y*start_node.theta) != 0.)
								flag_2 = 1;
							else
								flag_2 = 2;
						}
					}
					else
					{
						flag_1 = 2;
						if ((sin(end_node.theta - sign_y*beita_tmp) == 0.&&sin(alfa_tmp - sign_y*start_node.theta) == 0.) || (cos(end_node.theta - sign_y*beita_tmp) == 0.&&cos(alfa_tmp - sign_y*start_node.theta) == 0.))
							flag_2 = 0;
						else
						{
							if (sin(end_node.theta - sign_y*beita_tmp) == 0.&&sin(alfa_tmp - sign_y*start_node.theta) != 0.)
								flag_2 = 1;
							else
								flag_2 = 2;
						}
					}
				}

				while (result == false)
				{
					l4_tmp += step;

					x3_tmp = end_node.x - l4_tmp*cos(end_node.theta);
					y3_tmp = end_node.y - l4_tmp*sin(end_node.theta);
					if (sign_x*sign_y*y3_tmp < sign_x*sign_y*start_node.y - exceed_allow_x)
						break;

					// 第一种情况：C在D下方
					if (flag_1 != 0)
					{
						if (flag_1 == 1)
						{
							l2_tmp = (y3_tmp - y1_tmp) / sin(alfa_tmp - sign_y*start_node.theta);
							l3_tmp = (x1_tmp - x3_tmp - l2_tmp*cos(alfa_tmp - sign_y*start_node.theta)) / cos(end_node.theta + sign_y*beita_tmp);
						}
						else
						{
							l2_tmp = ((x1_tmp - x3_tmp)*sin(end_node.theta + sign_y*beita_tmp) + (y3_tmp - y1_tmp)*cos(end_node.theta + sign_y*beita_tmp)) / sin(alfa_tmp - sign_y*start_node.theta + end_node.theta + beita_tmp*sign_y);
							l3_tmp = (y1_tmp - y3_tmp + l2_tmp*sin(alfa_tmp - sign_y*start_node.theta)) / sin(end_node.theta + sign_y*beita_tmp);
						}

						if (l2_tmp > 0 && l3_tmp > 0)
						{
							if (std::min(l1_tmp, l2_tmp) > *(a + 1) && std::min(l4_tmp, l3_tmp) > *(b + 1))
							{
								gama_tmp = acos((pow(l2_tmp, 2) + pow(l3_tmp, 2) - pow(dist(x1_tmp, y1_tmp, x3_tmp, y3_tmp), 2)) / (2 * l2_tmp*l3_tmp));
								if (std::min(l2_tmp, l3_tmp) > Vehicle::_L_min(gama_tmp, 0))
								{
									x2_tmp = x1_tmp - l2_tmp*cos(alfa_tmp - sign_y*start_node.theta);
									y2_tmp = y1_tmp + l2_tmp*sin(alfa_tmp - sign_y*start_node.theta);
									if ((y1_tmp - start_node.y) <= -sign_x*exceed_allow_y*sign_y && (y2_tmp - start_node.y) <= -sign_x*exceed_allow_y*sign_y/* && (y3_tmp - start_node.y) <= -exceed_allow_y*sign && (std::max(x3_tmp, x2_tmp) - end_node.x) <= exceed_allow_y*/)
									{
										result = true;
										Vector7d optdata;
										optdata << min({ l1_tmp, l2_tmp, l3_tmp, l4_tmp }), x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
										opt->emplace_back(optdata);
										break;
									}
									else
									{
										suboptimal = true;
										delta_tmp = std::abs(y1_tmp - start_node.y) + std::abs(y2_tmp - start_node.y);
										if (delta_tmp < delta_subopt)
										{
											Vector7d suboptdata;
											suboptdata << min({ l1_tmp, l2_tmp, l3_tmp, l4_tmp }), x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
											subopt->emplace_back(suboptdata);
										}
									}
								}
							}
						}
					}
					
					//第二种情况：D在C下方
					if (flag_2 != 0)
					{
						if (flag_2 == 1)
						{
							l2_tmp = (y3_tmp - y1_tmp) / sin(alfa_tmp - sign_y*start_node.theta);
							l3_tmp = (x1_tmp - x3_tmp - l2_tmp*cos(alfa_tmp - sign_y*start_node.theta)) / cos(end_node.theta - sign_y*beita_tmp);
						}
						else
						{
							l2_tmp = ((x1_tmp - x3_tmp)*sin(end_node.theta - sign_y*beita_tmp) + (y3_tmp - y1_tmp)*cos(end_node.theta - sign_y*beita_tmp)) / sin(alfa_tmp - sign_y*start_node.theta + end_node.theta - beita_tmp*sign_y);
							l3_tmp = (y1_tmp - y3_tmp + l2_tmp*sin(alfa_tmp - sign_y*start_node.theta)) / sin(end_node.theta - sign_y*beita_tmp);
						}

						if (l2_tmp <= 0 || l3_tmp <= 0)
							continue;

						if (std::min(l1_tmp, l2_tmp) < *(a + 1) || std::min(l4_tmp, l3_tmp) < *(b + 1))
							continue;
						gama_tmp = acos((pow(l2_tmp, 2) + pow(l3_tmp, 2) - pow(dist(x1_tmp, y1_tmp, x3_tmp, y3_tmp), 2)) / (2 * l2_tmp*l3_tmp));
						if (std::min(l2_tmp, l3_tmp) < Vehicle::_L_min(gama_tmp, 0))
							continue;

						x2_tmp = x1_tmp - l2_tmp*cos(alfa_tmp - sign_y*start_node.theta);
						y2_tmp = y1_tmp + l2_tmp*sin(alfa_tmp - sign_y*start_node.theta);
						if ((y1_tmp - start_node.y) <= -sign_x*exceed_allow_y*sign_y && (y2_tmp - start_node.y) <= -sign_x*exceed_allow_y*sign_y/* && (y3_tmp - start_node.y) <= -exceed_allow_y*sign && (std::max(x3_tmp, x2_tmp) - end_node.x) <= exceed_allow_y*/)
						{
							result = true;
							Vector7d optdata;
							optdata << min({ l1_tmp, l2_tmp, l3_tmp, l4_tmp }), x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
							opt->emplace_back(optdata);
							break;
						}
						else
						{
							suboptimal = true;
							delta_tmp = std::abs(y1_tmp - start_node.y) + std::abs(y2_tmp - start_node.y);
							if (delta_tmp < delta_subopt)
							{
								Vector7d suboptdata;
								suboptdata << min({ l1_tmp, l2_tmp, l3_tmp, l4_tmp }), x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
								subopt->emplace_back(suboptdata);
							}
						}
					}
				}
			}
		}					
	}

	auto min_i = opt->begin();
	double min = 1000;
	if (result == true)
	{
		auto i = opt->begin();
		for (; i != opt->end(); i++)
		{
			if (min > (*i)[0])
			{
				min_i = i;
				min = (*i)[0];
			}
		}
	}
	else
	{
		if (suboptimal == true)
		{
			auto i = subopt->begin();
			for (; i != subopt->end(); i++)
			{
				if (min > (*i)[0])
				{
					min_i = i;
					min = (*i)[0];
				}
			}
			result = true;
		}
		else
			return false;
	}
	control->push_back(start_node.x); control->push_back(start_node.y);
	control->push_back((*min_i)[1]); control->push_back((*min_i)[2]);
	control->push_back((*min_i)[3]); control->push_back((*min_i)[4]);
	control->push_back((*min_i)[5]); control->push_back((*min_i)[6]);
	control->push_back(end_node.x); control->push_back(end_node.y);
	delete subopt, opt;
	return result;
}

bool Trajectory::turn_c(const Vehicle::Node &start_node, const Vehicle::Node &end_node, const vector<double> &constraints, vector<double> *control)
{
	control->clear();

	int sign_y;
	int sign_x;
	if ((end_node.x - start_node.x) < 0)
	{
		sign_x = -1;
		if ((end_node.y - start_node.y) > 0)
			sign_y = -1;
		else
			sign_y = 1;
	}
	else
	{
		sign_x = 1;
		if ((end_node.y - start_node.y) < 0)
			sign_y = -1;
		else
			sign_y = 1;
	}

	double alfa_tmp, beita_tmp, gama_tmp, l1_tmp, l2_tmp, l3_tmp, l4_tmp, x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
	double delta_subopt = 1000., delta_tmp;
	vector<Vector7d> *subopt = new vector<Vector7d>;
	vector<Vector7d> *opt = new vector<Vector7d>;
	bool result = false;
	bool suboptimal = false;
	int flag_1, flag_2; //分别为两种情况的l2和l3的计算，0表示无解，1为有解中有一个分母为0的情况，2为通用情况

	double l = constraints[0]; //从后轴中心算起到约束头
	double d = constraints[1]; //有正负

	double step = 0.5;
	double exceed_allow_x = 0;
	double exceed_allow_y = 2; //超出部分到车道中心线的距离

	for (auto a = L_theta.begin(); a != L_theta.end() - 4; a += 2)
	{
		if (result == true)
			break;
		// calculate l1
		alfa_tmp = *a;
		l1_tmp = *(a + 1) - step;

		while (result == false)
		{
			l1_tmp += step;

			// calculate l2&l3
			x1_tmp = start_node.x + l1_tmp*cos(start_node.theta);
			y1_tmp = start_node.y + l1_tmp*sin(start_node.theta);
			if (sign_x*(x1_tmp - end_node.x) > exceed_allow_x)
				break;

			for (auto b = L_theta.begin(); b != L_theta.end(); b += 2)
			{
				if (result == true)
					break;
				beita_tmp = *b;
				l4_tmp = *(b + 1) - step;

				if ((sin(end_node.theta + sign_y*beita_tmp) == 0.&&sin(alfa_tmp - sign_y*start_node.theta) == 0.) || (cos(end_node.theta + sign_y*beita_tmp) == 0.&&cos(alfa_tmp - sign_y*start_node.theta) == 0.))
				{
					if ((sin(end_node.theta - sign_y*beita_tmp) == 0.&&sin(alfa_tmp - sign_y*start_node.theta) == 0.) || (cos(end_node.theta - sign_y*beita_tmp) == 0.&&cos(alfa_tmp - sign_y*start_node.theta) == 0.))
						continue;
					else
					{
						flag_1 = 0;
						if (sin(end_node.theta - sign_y*beita_tmp) == 0.&&sin(alfa_tmp - sign_y*start_node.theta) != 0.)
							flag_2 = 1;
						else
							flag_2 = 2;
					}
				}
				else
				{
					if (sin(end_node.theta + sign_y*beita_tmp) == 0.&&sin(alfa_tmp - sign_y*start_node.theta) != 0.)
					{
						flag_1 = 1;
						if ((sin(end_node.theta - sign_y*beita_tmp) == 0.&&sin(alfa_tmp - sign_y*start_node.theta) == 0.) || (cos(end_node.theta - sign_y*beita_tmp) == 0.&&cos(alfa_tmp - sign_y*start_node.theta) == 0.))
							flag_2 = 0;
						else
						{
							if (sin(end_node.theta - sign_y*beita_tmp) == 0.&&sin(alfa_tmp - sign_y*start_node.theta) != 0.)
								flag_2 = 1;
							else
								flag_2 = 2;
						}
					}
					else
					{
						flag_1 = 2;
						if ((sin(end_node.theta - sign_y*beita_tmp) == 0.&&sin(alfa_tmp - sign_y*start_node.theta) == 0.) || (cos(end_node.theta - sign_y*beita_tmp) == 0.&&cos(alfa_tmp - sign_y*start_node.theta) == 0.))
							flag_2 = 0;
						else
						{
							if (sin(end_node.theta - sign_y*beita_tmp) == 0.&&sin(alfa_tmp - sign_y*start_node.theta) != 0.)
								flag_2 = 1;
							else
								flag_2 = 2;
						}
					}
				}

				while (result == false)
				{
					l4_tmp += step;

					x3_tmp = end_node.x - l4_tmp*cos(end_node.theta);
					y3_tmp = end_node.y - l4_tmp*sin(end_node.theta);
					if (sign_x*sign_y*y3_tmp < sign_x*sign_y*start_node.y - exceed_allow_x)
						break;

					// 第一种情况：C在D下方
					if (flag_1 != 0)
					{
						if (flag_1 == 1)
						{
							l2_tmp = (y3_tmp - y1_tmp) / sin(alfa_tmp - sign_y*start_node.theta);
							l3_tmp = (x1_tmp - x3_tmp - l2_tmp*cos(alfa_tmp - sign_y*start_node.theta)) / cos(end_node.theta + sign_y*beita_tmp);
						}
						else
						{
							l2_tmp = ((x1_tmp - x3_tmp)*sin(end_node.theta + sign_y*beita_tmp) + (y3_tmp - y1_tmp)*cos(end_node.theta + sign_y*beita_tmp)) / sin(alfa_tmp - sign_y*start_node.theta + end_node.theta + beita_tmp*sign_y);
							l3_tmp = (y1_tmp - y3_tmp + l2_tmp*sin(alfa_tmp - sign_y*start_node.theta)) / sin(end_node.theta + sign_y*beita_tmp);
						}

						if (l2_tmp > 0 && l3_tmp > 0)
						{
							if (std::min(l1_tmp, l2_tmp) > *(a + 1) && std::min(l4_tmp, l3_tmp) > *(b + 1))
							{
								gama_tmp = acos((pow(l2_tmp, 2) + pow(l3_tmp, 2) - pow(dist(x1_tmp, y1_tmp, x3_tmp, y3_tmp), 2)) / (2 * l2_tmp*l3_tmp));
								if (std::min(l2_tmp, l3_tmp) > Vehicle::_L_min(gama_tmp, 0))
								{
									x2_tmp = x1_tmp - l2_tmp*cos(alfa_tmp - sign_y*start_node.theta);
									y2_tmp = y1_tmp + l2_tmp*sin(alfa_tmp - sign_y*start_node.theta);
									
									suboptimal = true;
									if (sin(start_node.theta) != 0)
									{
										if ((l*std::abs(sin(start_node.theta)) + 0.5*W) >= std::abs(d*cos(start_node.theta)* +ERROR_3))
											suboptimal = false;
										else
										{
											if (d > sign_x*sign_y*(y1_tmp - start_node.y))
											{
												if (((l - std::abs(x1_tmp - start_node.x))* std::abs(sin(alfa_tmp - sign_y*start_node.theta)) + 0.5*W) >= ((d - std::abs(y1_tmp - start_node.y))*std::abs(cos(alfa_tmp - sign_y*start_node.theta)) + ERROR_3))
													suboptimal = false;
												else
												{
													if (d > sign_x*sign_y*(y2_tmp - start_node.y))
													{
														if (((l - std::abs(x2_tmp - start_node.x))* std::abs(sin(end_node.theta + sign_y*beita_tmp)) + 0.5*W) >= ((d - std::abs(y2_tmp - start_node.y))*std::abs(cos(end_node.theta + sign_y*beita_tmp)) + ERROR_3))
															suboptimal = false;
														else
														{
															if (d > sign_x*sign_y*(y3_tmp - start_node.y))
															{
																if (((l - std::abs(x3_tmp - start_node.x))* std::abs(sin(end_node.theta)) + 0.5*W) >= ((d - std::abs(y3_tmp - start_node.y))*std::abs(cos(end_node.theta)) + ERROR_3))
																	suboptimal = false;
															}
														}
													}
												}
											}
											else
											{
												if ((l* std::abs(sin(start_node.theta)) + 0.5*W) >= (d*std::abs(cos(alfa_tmp - sign_y*start_node.theta)) + ERROR_3))
													suboptimal = false;
											}
										}
									}
										
									if (suboptimal == true)
									{
										if ((y1_tmp - start_node.y) <= -sign_x*exceed_allow_y*sign_y && (y2_tmp - start_node.y) <= -sign_x*exceed_allow_y*sign_y/* && (y3_tmp - start_node.y) <= -exceed_allow_y*sign && (std::max(x3_tmp, x2_tmp) - end_node.x) <= exceed_allow_y*/)
										{
											result = true;
											Vector7d optdata;
											optdata << min({ l1_tmp, l2_tmp, l3_tmp, l4_tmp }), x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
											opt->emplace_back(optdata);
											break;
										}
										else
										{
											delta_tmp = std::abs(y1_tmp - start_node.y) + std::abs(y2_tmp - start_node.y);
											if (delta_tmp < delta_subopt)
											{
												Vector7d suboptdata;
												suboptdata << min({ l1_tmp, l2_tmp, l3_tmp, l4_tmp }), x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
												subopt->emplace_back(suboptdata);
											}
										}
									}
								}
							}
						}
					}

					//第二种情况：D在C下方
					if (flag_2 != 0)
					{
						if (flag_2 == 1)
						{
							l2_tmp = (y3_tmp - y1_tmp) / sin(alfa_tmp - sign_y*start_node.theta);
							l3_tmp = (x1_tmp - x3_tmp - l2_tmp*cos(alfa_tmp - sign_y*start_node.theta)) / cos(end_node.theta - sign_y*beita_tmp);
						}
						else
						{
							l2_tmp = ((x1_tmp - x3_tmp)*sin(end_node.theta - sign_y*beita_tmp) + (y3_tmp - y1_tmp)*cos(end_node.theta - sign_y*beita_tmp)) / sin(alfa_tmp - sign_y*start_node.theta + end_node.theta - beita_tmp*sign_y);
							l3_tmp = (y1_tmp - y3_tmp + l2_tmp*sin(alfa_tmp - sign_y*start_node.theta)) / sin(end_node.theta - sign_y*beita_tmp);
						}

						if (l2_tmp <= 0 || l3_tmp <= 0)
							continue;

						if (std::min(l1_tmp, l2_tmp) < *(a + 1) || std::min(l4_tmp, l3_tmp) < *(b + 1))
							continue;
						gama_tmp = acos((pow(l2_tmp, 2) + pow(l3_tmp, 2) - pow(dist(x1_tmp, y1_tmp, x3_tmp, y3_tmp), 2)) / (2 * l2_tmp*l3_tmp));
						if (std::min(l2_tmp, l3_tmp) < Vehicle::_L_min(gama_tmp, 0))
							continue;

						x2_tmp = x1_tmp - l2_tmp*cos(alfa_tmp - sign_y*start_node.theta);
						y2_tmp = y1_tmp + l2_tmp*sin(alfa_tmp - sign_y*start_node.theta);

						if (sin(start_node.theta) != 0)
						{
							if ((l*std::abs(sin(start_node.theta)) + 0.5*W) >= std::abs(d*cos(start_node.theta)* +ERROR_3))
								continue;
							else
							{
								if (d > sign_x*sign_y*(y1_tmp - start_node.y))
								{
									if (((l - std::abs(x1_tmp - start_node.x))* std::abs(sin(alfa_tmp - sign_y*start_node.theta)) + 0.5*W) >= (((d - std::abs(y1_tmp - start_node.y)))*std::abs(cos(alfa_tmp - sign_y*start_node.theta)) + ERROR_3))
										continue;
									else
									{
										if (d > sign_x*sign_y*(y2_tmp - start_node.y))
										{
											if (((l - std::abs(x2_tmp - start_node.x))* std::abs(sin(end_node.theta + sign_y*beita_tmp)) + 0.5*W) >= (((d - std::abs(y2_tmp - start_node.y)))*std::abs(cos(end_node.theta + sign_y*beita_tmp)) + ERROR_3))
												continue;
											else
											{
												if (d > sign_x*sign_y*(y3_tmp - start_node.y))
												{
													if (((l - std::abs(x3_tmp - start_node.x))* std::abs(sin(end_node.theta)) + 0.5*W) >= (((d - std::abs(y3_tmp - start_node.y)))*std::abs(cos(end_node.theta)) + ERROR_3))
														continue;
												}
											}
										}
									}
								}
								else
								{
									if ((l* std::abs(sin(start_node.theta)) + 0.5*W) >= (d*std::abs(cos(alfa_tmp - sign_y*start_node.theta)) + ERROR_3))
										suboptimal = false;
								}
							}
						}

						if ((y1_tmp - start_node.y) <= -sign_x*exceed_allow_y*sign_y && (y2_tmp - start_node.y) <= -sign_x*exceed_allow_y*sign_y/* && (y3_tmp - start_node.y) <= -exceed_allow_y*sign && (std::max(x3_tmp, x2_tmp) - end_node.x) <= exceed_allow_y*/)
						{
							result = true;
							Vector7d optdata;
							optdata << min({ l1_tmp, l2_tmp, l3_tmp, l4_tmp }), x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
							opt->emplace_back(optdata);
							break;
						}
						else
						{
							suboptimal = true;
							delta_tmp = std::abs(y1_tmp - start_node.y) + std::abs(y2_tmp - start_node.y);
							if (delta_tmp < delta_subopt)
							{
								Vector7d suboptdata;
								suboptdata << min({ l1_tmp, l2_tmp, l3_tmp, l4_tmp }), x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
								subopt->emplace_back(suboptdata);
							}
						}
					}
				}
			}
		}
	}

	auto min_i = opt->begin();
	double min = 1000;
	if (result == true)
	{
		auto i = opt->begin();
		for (; i != opt->end(); i++)
		{
			if (min > (*i)[0])
			{
				min_i = i;
				min = (*i)[0];
			}
		}
	}
	else
	{
		if (suboptimal == true)
		{
			auto i = subopt->begin();
			for (; i != subopt->end(); i++)
			{
				if (min > (*i)[0])
				{
					min_i = i;
					min = (*i)[0];
				}
			}
			result = true;
		}
		else
			return false;
	}
	control->push_back(start_node.x); control->push_back(start_node.y);
	control->push_back((*min_i)[1]); control->push_back((*min_i)[2]);
	control->push_back((*min_i)[3]); control->push_back((*min_i)[4]);
	control->push_back((*min_i)[5]); control->push_back((*min_i)[6]);
	control->push_back(end_node.x); control->push_back(end_node.y);
	delete subopt, opt;
	return result;
}

bool Trajectory::U_turn(const Vehicle::Node &start_node, const Vehicle::Node &end_node, vector<double> *control)
{
	control->clear();

	int sign_x;
	if ((end_node.x - start_node.x) < 0)
		sign_x = -1;
	else
		sign_x = 1;

	double alfa_tmp, beita_tmp, gama_tmp, l1_tmp, l2_tmp, l3_tmp, l4_tmp, x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
	bool result = false;
	bool suboptimal = false;
	vector<Vector7d>* subopt = new vector<Vector7d>;
	vector<Vector7d>* opt = new vector<Vector7d>;

	double step = 0.2;
	double exceed_allow_x = 10.;
	double exceed_allow_y = 2; //超出部分到车道中心线的距离
	for (auto a = L_theta.begin(); a != L_theta.end() - 4; a += 2)
	{
		if (result == true)
			break;
		// calculate l1
		alfa_tmp = *a;
		l1_tmp = *(a + 1) - step;

		while (result == false)
		{
			l1_tmp += step;

			// calculate l2&l3
			x1_tmp = start_node.x + l1_tmp*cos(start_node.theta);
			y1_tmp = start_node.y + l1_tmp*sin(start_node.theta);
			if (sign_x*(x1_tmp - end_node.x) > exceed_allow_x)
				break;
			if (y1_tmp<YMIN || y1_tmp>YMAX)
				break;

			for (auto b = L_theta.begin(); b != L_theta.end(); b += 2)
			{
				if (result == true)
					break;
				beita_tmp = *b;
				l4_tmp = *(b + 1) - step;

				if ((sin(end_node.theta + beita_tmp) == 0.&&sin(alfa_tmp - start_node.theta) == 0.) || (cos(end_node.theta + beita_tmp) == 0.&&cos(alfa_tmp - start_node.theta) == 0.))
					continue;

				while (result == false)
				{
					l4_tmp += step;

					x3_tmp = end_node.x - l4_tmp*cos(end_node.theta);
					y3_tmp = end_node.y - l4_tmp*sin(end_node.theta);
					if (sign_x*y3_tmp < sign_x*start_node.y - exceed_allow_x || sign_x*(x3_tmp - end_node.x) > exceed_allow_x)
						break;

					if (sin(end_node.theta + beita_tmp) == 0 && sin(alfa_tmp - start_node.theta) != 0)
					{
						l2_tmp = (y3_tmp - y1_tmp) / sin(alfa_tmp - start_node.theta);
						l3_tmp = (x1_tmp - x3_tmp - l2_tmp*cos(alfa_tmp - start_node.theta)) / cos(end_node.theta + beita_tmp);
					}
					else
					{
						l2_tmp = ((x1_tmp - x3_tmp)*sin(end_node.theta + beita_tmp) + (y3_tmp - y1_tmp)*cos(end_node.theta + beita_tmp)) / sin(alfa_tmp - start_node.theta + end_node.theta + beita_tmp);
						l3_tmp = (y1_tmp - y3_tmp + l2_tmp*sin(alfa_tmp - start_node.theta)) / sin(end_node.theta + beita_tmp);
					}

					if (l2_tmp <= 0 || l3_tmp <= 0)
						continue;

					if (std::min(l1_tmp, l2_tmp) < *(a + 1) || std::min(l4_tmp, l3_tmp) < *(b + 1))
						continue;
					gama_tmp = acos((pow(l2_tmp, 2) + pow(l3_tmp, 2) - pow(dist(x1_tmp, y1_tmp, x3_tmp, y3_tmp), 2)) / (2 * l2_tmp*l3_tmp));
					if (std::min(l2_tmp, l3_tmp) < Vehicle::_L_min(gama_tmp, 0))
						continue;

					x2_tmp = x1_tmp - l2_tmp*cos(alfa_tmp - start_node.theta);
					y2_tmp = y1_tmp + l2_tmp*sin(alfa_tmp - start_node.theta);

					if (sign_x*(x2_tmp - end_node.x) > exceed_allow_x)
						continue;

					if ((y1_tmp - start_node.y) <= -sign_x*exceed_allow_y && (y2_tmp - start_node.y) <= -sign_x*exceed_allow_y /*&& (y3_tmp - start_node.y) <= -exceed_allow_y*sign && (std::max(x3_tmp, x2_tmp) - end_node.x) <= exceed_allow_y*/)
					{
						result = true;
						Vector7d optdata;
						optdata << min({ l1_tmp, l2_tmp, l3_tmp, l4_tmp }), x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
						opt->emplace_back(optdata);
						break;
					}
					else
					{
						suboptimal = true;
						Vector7d suboptdata;
						suboptdata << min({ l1_tmp, l2_tmp, l3_tmp, l4_tmp }), x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
						subopt->emplace_back(suboptdata);
					}
				}
			}
		}
	}

	auto min_i = opt->begin();
	double min = 1000;
	if (result == true)
	{
		auto i = opt->begin();
		for (; i != opt->end(); i++)
		{
			if (min > (*i)[0])
			{
				min_i = i;
				min = (*i)[0];
			}
		}
	}
	else
	{
		if (suboptimal == true)
		{
			auto i = subopt->begin();
			for (; i != subopt->end(); i++)
			{
				if (min > (*i)[0])
				{
					min_i = i;
					min = (*i)[0];
				}
			}
			result = true;
		}
		else
			return false;
	}
	control->push_back(start_node.x); control->push_back(start_node.y);
	control->push_back((*min_i)[1]); control->push_back((*min_i)[2]);
	control->push_back((*min_i)[3]); control->push_back((*min_i)[4]);
	control->push_back((*min_i)[5]); control->push_back((*min_i)[6]);
	control->push_back(end_node.x); control->push_back(end_node.y);
	delete subopt, opt;
	return result;
}

bool Trajectory::U_turn_c(const Vehicle::Node &start_node, const Vehicle::Node &end_node, const vector<double> &constraints, vector<double> *control)
{
	control->clear();

	int sign_x;
	if ((end_node.x - start_node.x) < 0)
		sign_x = -1;
	else
		sign_x = 1;

	double alfa_tmp, beita_tmp, gama_tmp, l1_tmp, l2_tmp, l3_tmp, l4_tmp, x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
	bool result = false;
	bool suboptimal = false;
	vector<Vector7d>* subopt = new vector<Vector7d>;
	vector<Vector7d>* opt = new vector<Vector7d>;

	double l = constraints[0];
	double d = constraints[1];

	double step = 0.2;
	double exceed_allow_x = 10.;
	double exceed_allow_y = 2; //超出部分到车道中心线的距离
	for (auto a = L_theta.begin(); a != L_theta.end() - 4; a += 2)
	{
		if (result == true)
			break;
		// calculate l1
		alfa_tmp = *a;
		l1_tmp = *(a + 1) - step;

		while (result == false)
		{
			l1_tmp += step;

			// calculate l2&l3
			x1_tmp = start_node.x + l1_tmp*cos(start_node.theta);
			y1_tmp = start_node.y + l1_tmp*sin(start_node.theta);
			if (sign_x*(x1_tmp - end_node.x) > exceed_allow_x)
				break;
			if (y1_tmp<YMIN || y1_tmp>YMAX)
				break;

			for (auto b = L_theta.begin(); b != L_theta.end(); b += 2)
			{
				if (result == true)
					break;
				beita_tmp = *b;
				l4_tmp = *(b + 1) - step;

				if ((sin(end_node.theta + beita_tmp) == 0.&&sin(alfa_tmp - start_node.theta) == 0.) || (cos(end_node.theta + beita_tmp) == 0.&&cos(alfa_tmp - start_node.theta) == 0.))
					continue;

				while (result == false)
				{
					l4_tmp += step;

					x3_tmp = end_node.x - l4_tmp*cos(end_node.theta);
					y3_tmp = end_node.y - l4_tmp*sin(end_node.theta);
					if (sign_x*y3_tmp < sign_x*start_node.y - exceed_allow_x || sign_x*(x3_tmp - end_node.x) > exceed_allow_x)
						break;

					if (sin(end_node.theta + beita_tmp) == 0 && sin(alfa_tmp - start_node.theta) != 0)
					{
						l2_tmp = (y3_tmp - y1_tmp) / sin(alfa_tmp - start_node.theta);
						l3_tmp = (x1_tmp - x3_tmp - l2_tmp*cos(alfa_tmp - start_node.theta)) / cos(end_node.theta + beita_tmp);
					}
					else
					{
						l2_tmp = ((x1_tmp - x3_tmp)*sin(end_node.theta + beita_tmp) + (y3_tmp - y1_tmp)*cos(end_node.theta + beita_tmp)) / sin(alfa_tmp - start_node.theta + end_node.theta + beita_tmp);
						l3_tmp = (y1_tmp - y3_tmp + l2_tmp*sin(alfa_tmp - start_node.theta)) / sin(end_node.theta + beita_tmp);
					}

					if (l2_tmp <= 0 || l3_tmp <= 0)
						continue;

					if (std::min(l1_tmp, l2_tmp) < *(a + 1) || std::min(l4_tmp, l3_tmp) < *(b + 1))
						continue;
					gama_tmp = acos((pow(l2_tmp, 2) + pow(l3_tmp, 2) - pow(dist(x1_tmp, y1_tmp, x3_tmp, y3_tmp), 2)) / (2 * l2_tmp*l3_tmp));
					if (std::min(l2_tmp, l3_tmp) < Vehicle::_L_min(gama_tmp, 0))
						continue;

					x2_tmp = x1_tmp - l2_tmp*cos(alfa_tmp - start_node.theta);
					y2_tmp = y1_tmp + l2_tmp*sin(alfa_tmp - start_node.theta);

					if (sign_x*x2_tmp > sign_x*end_node.x + exceed_allow_x)
						continue;

					if (sin(start_node.theta) != 0)
					{
						if ((l*std::abs(sin(start_node.theta)) + 0.5*W) >= std::abs(d*cos(start_node.theta)* +ERROR_3))
							continue;
						else
						{
							if (d > sign_x*(y1_tmp - start_node.y))
							{
								if (sin(alfa_tmp - start_node.theta) != 0)
								{
									if (((l - std::abs(x1_tmp - start_node.x))* std::abs(sin(alfa_tmp - start_node.theta)) + 0.5*W) >= (((d - std::abs(y1_tmp - start_node.y)))*std::abs(cos(alfa_tmp - start_node.theta)) + ERROR_3))
										continue;
									else
									{
										if (d > sign_x*(y2_tmp - start_node.y))
										{
											if (sin(end_node.theta + beita_tmp) != 0)
											{
												if (((l - std::abs(x2_tmp - start_node.x))* std::abs(sin(end_node.theta + beita_tmp)) + 0.5*W) >= (((d - std::abs(y2_tmp - start_node.y)))*std::abs(cos(end_node.theta + beita_tmp)) + ERROR_3))
													continue;
												else
												{
													if (d > sign_x*(y3_tmp - start_node.y))
													{
														if (sin(end_node.theta) != 0)
														{
															if (((l - std::abs(x3_tmp - start_node.x))* std::abs(sin(end_node.theta)) + 0.5*W) >= (((d - std::abs(y3_tmp - start_node.y)))*std::abs(cos(end_node.theta)) + ERROR_3))
																continue;
														}
													}
												}
											}
										}
									}
								}
							}
						}
					}

					if ((y1_tmp - start_node.y) <= -sign_x*exceed_allow_y && (y2_tmp - start_node.y) <= -sign_x*exceed_allow_y/* && (y3_tmp - start_node.y) <= -exceed_allow_y*sign && (std::max(x3_tmp, x2_tmp) - end_node.x) <= exceed_allow_y*/)
					{
						result = true;
						Vector7d suboptdata;
						suboptdata << min({ l1_tmp, l2_tmp, l3_tmp, l4_tmp }), x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
						subopt->emplace_back(suboptdata);
						break;
					}
					else
					{
						suboptimal = true;
						Vector7d suboptdata;
						suboptdata << min({ l1_tmp, l2_tmp, l3_tmp, l4_tmp }), x1_tmp, y1_tmp, x2_tmp, y2_tmp, x3_tmp, y3_tmp;
						subopt->emplace_back(suboptdata);
					}
				}
			}
		}
	}

	auto min_i = opt->begin();
	double min = 1000;
	if (result == true)
	{
		auto i = opt->begin();
		for (; i != opt->end(); i++)
		{
			if (min > (*i)[0])
			{
				min_i = i;
				min = (*i)[0];
			}
		}
	}
	else
	{
		if (suboptimal == true)
		{
			auto i = subopt->begin();
			for (; i != subopt->end(); i++)
			{
				if (min > (*i)[0])
				{
					min_i = i;
					min = (*i)[0];
				}
			}
			result = true;
		}
	}
	control->push_back(start_node.x); control->push_back(start_node.y);
	control->push_back((*min_i)[1]); control->push_back((*min_i)[2]);
	control->push_back((*min_i)[3]); control->push_back((*min_i)[4]);
	control->push_back((*min_i)[5]); control->push_back((*min_i)[6]);
	control->push_back(end_node.x); control->push_back(end_node.y);
	delete subopt, opt;
	return result;
}

int Trajectory::is_force_extend_safe(const Vehicle::Node &startnode, const double &step, const vector<double> &control, double *le, vector<Vehicle::Node> *path)
{
	ts::BSpline bspline = ts::BSpline(3, 2, control.size() - 1, TS_CLAMPED);
	vector<ts::rational> ctrl_points = bspline.ctrlp();
	ctrl_points[0] = control[0];				ctrl_points[1] = control[1];
	for (int i = 2; i < ctrl_points.size(); i += 4)
	{
		ctrl_points[i] = 0.5*(control[1 + i / 2] + control[i / 2 - 1]);
		ctrl_points[i + 1] = 0.5*(control[2 + i / 2] + control[i / 2]);
		ctrl_points[i + 2] = control[1 + i / 2];
		ctrl_points[i + 3] = control[2 + i / 2];
	}
	bspline.setCtrlp(ctrl_points);  //在原控制点中插入中点生成所需要的B样条曲线
	int issafe = 2;
	double du = 0.01;
	double u = 0.;
	double s = 0.;
	int count = 0;
	*le = 0;
	double le_tmp = 0;
	int time = (int)std::floor(step / SAFESTEP);
	vector<double> old_tmp{ startnode.x, startnode.y };
	Point2D diff_1, diff_2, old_diff(0, 0);
	for (; u < 1.; u += du)
	{
		auto result = bspline.evaluate(u).result();
		diff_1.reset(result[0] - old_tmp[0], result[1] - old_tmp[1]);
		diff_2.reset(diff_1.x - old_diff.x, diff_1.y - old_diff.y);
		double theta = atan2(diff_1.y, diff_1.x);
		s += sqrt(pow(result[0] - old_tmp[0], 2.) + pow(result[1] - old_tmp[1], 2.));
		old_tmp[0] = result[0];          old_tmp[1] = result[1];
		old_diff = diff_1;
		if (s < SAFESTEP - ERROR_1)
			continue;
		if (collimap->iscollision(result[0], result[1], theta))
		{
			issafe = 1;
			break;
		}
		double k = (diff_1.x*diff_2.y - diff_2.x*diff_1.y) / pow(sqrt(pow(diff_1.x, 2.) + pow(diff_2.y, 2.)), 1.5);

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
		path->emplace_back(Vehicle::Node(ctrl_points[2 * i], ctrl_points[2 * i + 1], atan2(ctrl_points[2 * i + 1] - old_node.y, ctrl_points[2 * i] - old_node.x)/* + startnode.theta*/));
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