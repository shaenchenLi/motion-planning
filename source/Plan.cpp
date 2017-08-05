#include "Plan.h"

static PLAN_INPUT plan_input;
static PLAN_OUTPUT local_output;
static PLAN_INPUT last_input;
static int search_result = 0; //用来表示前一次搜索的结果
static int is_plan = 0; //0：不需要规划  1：需要搜索路径  2：只用规划速度  3：上次没有规划完成，这次需要边规划，边生成速度
static int curr_u;
static int replan = 0;

RRT::RTG_RRT tree, original_tree;
Trajectory::traj trajectory;

std::mutex mutex;//线程互斥对象

ofstream controlpoints, treenodes, route_tree, real_path, current, path_executed;
char controlname[100], pathname[100], treename[100], routename[100], currentname[100], path_executed_name[100];

void _init(const int &isLog)
{
	file<> fdoc("E:\\postgraduate\\codes\\PLAN\\plan_config.xml");
	xml_document<> doc;
	doc.parse<0>(fdoc.data());
	xml_node<>* root = doc.first_node();

	//Vehicle
	xml_node<>* V = root->first_node("Vehicle");
	xml_node<>* VL = V->first_node("L");
	L = double(atof(VL->value()));
	xml_node<>* VW = V->first_node("W");
	W = double(atof(VW->value()));
	xml_node<>* VLA = V->first_node("LA");
	LA = double(atof(VLA->value()));
	xml_node<>* VL_F_BA = V->first_node("L_F_BA");
	L_F_BA = double(atof(VL_F_BA->value()));
	xml_node<>* VKMAX = V->first_node("KMAX");
	KMAX = double(atof(VKMAX->value()));
	xml_node<>* VVMAX = V->first_node("VMAX");
	VMAX = double(atof(VVMAX->value()));
	xml_node<>* VAMIN = V->first_node("AMIN");
	AMIN = double(atof(VAMIN->value()));
	xml_node<>* VAMAX = V->first_node("AMAX");
	AMAX = double(atof(VAMAX->value()));
	xml_node<>* VALATER = V->first_node("ALATER");
	ALATER = double(atof(VALATER->value()));
	xml_node<>* VDFAIMAX = V->first_node("DFAIMAX");
	DFAIMAX = double(atof(VDFAIMAX->value()));
	xml_node<>* VALFA_MIN = V->first_node("ALFA_MIN");
	ALFA_MIN = double(atof(VALFA_MIN->value()));
	xml_node<>* VV_PRE = V->first_node("V_PRE");
	V_PRE = double(atof(VV_PRE->value()));

	//ERROR
	xml_node<>* E = root->first_node("ERROR");
	xml_node<>* EV = E->first_node("ERROR_V");
	ERROR_V = double(atof(EV->value()));
	xml_node<>* EO = E->first_node("ERROR_OBS");
	ERROR_OBS = double(atof(EO->value()));
	xml_node<>* EK = E->first_node("ERROR_KMAX");
	ERROR_KMAX = double(atof(EK->value()));
	xml_node<>* ES = E->first_node("ERROR_SAFE");
	ERROR_SAFE = double(atof(ES->value()));
	xml_node<>* EB = E->first_node("ERROR_BRAKE");
	ERROR_BRAKE = double(atof(EB->value()));
	xml_node<>* ET = E->first_node("ERROR_T");
	ERROR_T = double(atof(ET->value()));

	//RRT
	xml_node<>* R = root->first_node("RRT");
	xml_node<>* RRT_s = R->first_node("RRTSTEP_s");
	RRTSTEP_s = double(atof(RRT_s->value()));
	xml_node<>* RRT_l = R->first_node("RRTSTEP_l");
	RRTSTEP_l = double(atof(RRT_l->value()));
	xml_node<>* RRT_1 = R->first_node("MAXITER_1");
	MAXITER_1 = atoi(RRT_1->value());
	xml_node<>* RRT_2 = R->first_node("MAXITER_2");
	MAXITER_2 = atoi(RRT_2->value());
	xml_node<>* RRT_k = R->first_node("kNN_NUM");
	kNN_NUM = atoi(RRT_k->value());
	xml_node<>* RRT_b = R->first_node("BT_MAX");
	BT_MAX = atoi(RRT_b->value());
	xml_node<>* RRT_m = R->first_node("MAX_TRAJ_ITER");
	MAX_TRAJ_ITER = atoi(RRT_m->value());

	//TRAJECTORY
	xml_node<>* T = root->first_node("TRAJECTORY");
	xml_node<>* TP = T->first_node("POINTS_NUM");
	POINTS_NUM = atoi(TP->value());
	xml_node<>* TS = T->first_node("SAFESTEP");
	SAFESTEP = double(atof(TS->value()));
	xml_node<>* TPL = T->first_node("PLAN_PERIOD");
	PLAN_PERIOD = double(atof(TPL->value()));
	xml_node<>* TSS = T->first_node("STATIC_SAFE_LENGTH");
	STATIC_SAFE_LENGTH = double(atof(TSS->value()));
	xml_node<>* TSL = T->first_node("SAFE_LENGTH_MARGIN");
	SAFE_LENGTH_MARGIN = double(atof(TSL->value()));

	L_theta = Vehicle::ensure_k();

	if (isLog)
	{
		char dirname[100];
		time_t tt = time(NULL);
		tm* t = localtime(&tt);
		sprintf_s(dirname, "..\\..\\validate\\data%4d.%02d.%02d@%02d.%02d.%02d", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
		system(("md " + std::string(dirname)).c_str());
		sprintf_s(controlname, "%s\\controlpoints.txt", dirname);
		sprintf_s(treename, "%s\\treenodes.txt", dirname);
		sprintf_s(routename, "%s\\route_tree.txt", dirname);
		sprintf_s(pathname, "%s\\path.txt", dirname);
		sprintf_s(currentname, "%s\\current.txt", dirname);
		sprintf_s(path_executed_name, "%s\\path_executed.txt", dirname);
	}
}

void is_to_plan()
{	
	if (plan_input.flag != 0)
	{
		if (last_input == plan_input)
		{
			if (last_input.flag == plan_input.flag)
			{
				if (local_output.success == 1)
				{
					if (replan == 1)
					{
						is_plan = 1;
						//重规划是否需要我自己重新确定输入还是决策重新给输入？
						plan_input.XI.get()->_node(*(*trajectory._entire_traj())[u_begin + curr_u]._node());
						plan_input.obs.get()->clear();
					}
					else
						is_plan = 2;
				}
				else
					is_plan = 3;
			}
			else
				is_plan = 1;
		}
		else
		{
			is_plan = 1;
		}
	}
	
	if (is_plan == 1) //重新规划速度，因此序号重置
	{
		entire_traj_num = 0;
		u_begin = 0;
		curr_u = 0;
	}

	last_input = plan_input;
}

void _no_local_plan()
{
	for (int i = 0; i < S_SEGMENT; i++)
		(*local_output.x)[i] = (*local_output.y)[i] = (*local_output.v)[i] = (*local_output.k)[i] = 0;

	local_output.is_local = 0;
	local_output.local_to_local = 0;
	local_output.success = 0;
	local_output.XG_planned.reset();

	curr_u = 0;
}

void search_path()
{
	int flag = plan_input.flag;
	Trajectory::State state_i = Trajectory::State(*plan_input.XI.get());
	Trajectory::State state_g = Trajectory::State(*plan_input.XG.get());

	double road_angle = plan_input.road_angle;
	
	vector<double> constraint = *plan_input.obs.get();

	search_result = 0;
	tree.reset(state_i, state_g);
	int result = tree.search(flag, constraint, road_angle);
	trajectory.reset(state_i, state_g);

	search_result = result;
	if (result != 0)
	{
		tree.getpath(tree._dest()->_index());
		if (trajectory._path(result, *tree._route_tree()))
		{
			local_output.success = 1;
		}
		else
		{
			search_result = 0;
			local_output.success = -1;
		}
	}
	else
	{
		local_output.success = -1;
	}
}

void plan_velocity()
{
	mutex.lock();
	replan = Trajectory::_output(trajectory._entire_traj(), *trajectory._u_div(), *trajectory._v_div(), curr_u, local_output.x.get(), local_output.y.get(), local_output.k.get(), local_output.v.get());
	mutex.unlock();
}

void search_path_append()
{
	original_tree._copy(tree);

	int result_new;
	vector<RRT::Index> NN;
	RRT::Index new_root;

	tree.near_search(*tree._dest()->_node(), &NN);
	new_root = *NN.rbegin();
	NN.pop_back();
	tree.reset(new_root);
	result_new = tree.search(0);

	while (tree._tree()->size() == 1 && result_new == 0)
	{
		if (!NN.empty())
		{
			new_root = *NN.rbegin();
			NN.pop_back();
		}
		else
		{
			result_new = -1;
			break;
		}
		tree._copy(original_tree);
		tree.reset(new_root);
		result_new = tree.search(0);
	}

	mutex.lock();
	original_tree.getpath(new_root);
	if (original_tree._route_tree()->size() == 1)
		result_new = 0;
	else
		trajectory._path(search_result, *original_tree._route_tree());
	mutex.unlock();

	search_result = result_new;
}

void _plan(const int &isLog, const PLAN_INPUT &input, PLAN_OUTPUT *output, const int &u, const double &t, const double &v, const double &a, const int &is_new_map)
{
	curr_u = u;
	curr_t = t;
	curr_v = v;
	curr_a = a;

	if (is_new_map)
		map_origin = *(*trajectory._entire_traj())[curr_u + u_begin]._node();
	else if (u_begin + curr_u == 0)
		map_origin = *input.XI.get()->_node();

	memcpy(&plan_input, &input, sizeof(PLAN_INPUT));

	auto start = steady_clock::now();

	is_to_plan();

	system_clock::time_point end1;
	switch (is_plan)
	{
	case 0:
		_no_local_plan();
		break;
	case 1:
		search_path();
		end1 = steady_clock::now();
		cout << (end1 - start).count()*pow(10, -4) << endl;
		plan_velocity();
		break;
	case 2:
		plan_velocity();
		break;
	case 3:
		std::thread vel(plan_velocity);
		std::thread path(search_path_append);
		//开两个线程，一个算速度，一个算路径
		vel.join();
		path.join();
	}

	output->x = local_output.x;
	output->y = local_output.y;
	output->k = local_output.k;
	output->v = local_output.v;

	auto end = steady_clock::now();
	cout << (end - start).count()*pow(10, -4) << endl;

	if (isLog)
	{
		_log();
	}
}

void _log()
{
	static int path_num = 1; //用来判断现在输出的路径是第几段路径
	static int plan_num = 0;

	switch (is_plan)
	{
	case 3:
		path_num++;
	case 1:
		controlpoints.open(controlname, std::ios::app);
		treenodes.open(treename, std::ios::app);
		route_tree.open(routename, std::ios::app);

		for (auto it : *trajectory._ctrl_points())
			controlpoints << path_num << "," << it.x << "," << it.y << "," << endl;

		for (auto &it : *tree._route_tree())
			route_tree << path_num << "," << it.x << "," << it.y << "," << it.theta << endl;
		
		for (auto it : *tree._tree())
			treenodes << path_num << "," << it._node()->x << " " << it._node()->y << " " << endl;

		treenodes.close();
		route_tree.close();
		controlpoints.close();
	case 2:
		plan_num++;

		real_path.open(pathname, std::ios::app);
		current.open(currentname, std::ios::app); //用来储存当前轨迹点

		int m = local_output.x->size();
		for (int i = 0; i < m; i++)
			real_path << plan_num << "," << (*local_output.x.get())[i] << "," << (*local_output.y.get())[i] << "," << (*local_output.k.get())[i] << "," << (*local_output.v.get())[i] << endl;

		if (local_change == 1)
			current << plan_num << "," << u_begin << "," << curr_v << endl;
		else
			current << plan_num << "," << u_begin + curr_u << "," << curr_v << endl;

		real_path.close();
		current.close();
	}
}