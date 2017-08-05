#include "Configuration.h"

double RRTSTEP_s(0.), RRTSTEP_l(0.);
int MAXITER_1(0), MAXITER_2(0);
int kNN_NUM(0);
int BT_MAX(0);
int MAX_TRAJ_ITER(0);

double L(0.), W(0.), LA(0.), L_F_BA(0.), KMAX(0.), VMAX(0.), AMIN(0.), AMAX(0.), ALATER(0.), DFAIMAX(0.), ALFA_MIN(0.), V_PRE(0.), DL(0.);

double ERROR_SAFE(0.), ERROR_OBS(0.), ERROR_KMAX(0.), ERROR_V(0.), ERROR_BRAKE(0.), ERROR_T(0.);

double SAFESTEP(0.), POINTS_NUM(0.), PLAN_PERIOD(0.), STATIC_SAFE_LENGTH(0.), SAFE_LENGTH_MARGIN(0.);

void read_config()
{
	file<> fdoc("E:\\postgraduate\\codes\\PLAN\\plan_config.xml");
	xml_document<> doc;
	doc.parse<0>(fdoc.data());
	xml_node<>* root = doc.first_node();

	xml_node<>* R = root->first_node("RRT");
	xml_node<>* RRTSTEP_s_xml = R->first_node("RRTSTEP_s");
	RRTSTEP_s = double(atof(RRTSTEP_s_xml->value()));
	xml_node<>* RRTSTEP_l_xml = R->first_node("RRTSTEP_l");
	RRTSTEP_l = double(atof(RRTSTEP_l_xml->value()));
	xml_node<>* MAXITER_1_xml = R->first_node("MAXITER_1");
	MAXITER_1 = atoi(MAXITER_1_xml->value());
	xml_node<>* MAXITER_2_xml = R->first_node("MAXITER_2");
	MAXITER_2 = atoi(MAXITER_2_xml->value());
	xml_node<>* kNN_NUM_xml = R->first_node("kNN_NUM");
	kNN_NUM = atoi(kNN_NUM_xml->value());
	xml_node<>* BT_MAX_xml = R->first_node("BT_MAX");
	BT_MAX = atoi(BT_MAX_xml->value());
	xml_node<>* MAX_TRAJ_ITER_xml = R->first_node("MAX_TRAJ_ITER");
	MAX_TRAJ_ITER = atoi(MAX_TRAJ_ITER_xml->value());

	xml_node<>* V = root->first_node("Vehicle");
	xml_node<>* L_xml = V->first_node("L");
	L = double(atof(L_xml->value()));
	xml_node<>* W_xml = V->first_node("W");
	W = double(atof(W_xml->value()));
	xml_node<>* LA_xml = V->first_node("LA");
	LA = double(atof(LA_xml->value()));
	xml_node<>* L_F_BA_xml = V->first_node("L_F_BA");
	L_F_BA = double(atof(L_F_BA_xml->value()));
	xml_node<>* KMAX_xml = V->first_node("KMAX");
	KMAX = double(atof(KMAX_xml->value()));
	xml_node<>* VMAX_xml = V->first_node("VMAX");
	VMAX = double(atof(VMAX_xml->value()));
	xml_node<>* AMIN_xml = V->first_node("AMIN");
	AMIN = double(atof(AMIN_xml->value()));
	xml_node<>* AMAX_xml = V->first_node("AMAX");
	AMAX = double(atof(AMAX_xml->value()));
	xml_node<>* ALATER_xml = V->first_node("ALATER");
	ALATER = double(atof(ALATER_xml->value()));
	xml_node<>* DFAIMAX_xml = V->first_node("DFAIMAX");
	DFAIMAX = double(atof(DFAIMAX_xml->value()));
	xml_node<>* ALFA_MIN_xml = V->first_node("ALFA_MIN");
	ALFA_MIN = double(atof(ALFA_MIN_xml->value()));
	xml_node<>* V_PRE_xml = V->first_node("V_PRE");
	V_PRE = double(atof(V_PRE_xml->value()));

	xml_node<>* E = root->first_node("ERROR");
	xml_node<>* ERROR_V_xml = E->first_node("ERROR_V");
	ERROR_V = double(atof(ERROR_V_xml->value()));
	xml_node<>* ERROR_OBS_xml = E->first_node("ERROR_OBS");
	ERROR_OBS = double(atof(ERROR_OBS_xml->value()));
	xml_node<>* ERROR_SAFE_xml = E->first_node("ERROR_SAFE");
	ERROR_SAFE = double(atof(ERROR_SAFE_xml->value()));
	xml_node<>* ERROR_KMAX_xml = E->first_node("ERROR_KMAX");
	ERROR_KMAX = double(atof(ERROR_KMAX_xml->value()));
	xml_node<>* ERROR_BRAKE_xml = E->first_node("ERROR_BRAKE");
	ERROR_BRAKE = double(atof(ERROR_BRAKE_xml->value()));
	xml_node<>* ERROR_T_xml = E->first_node("ERROR_T");
	ERROR_T = double(atof(ERROR_T_xml->value()));

	xml_node<>* T = root->first_node("TRAJECTORY");
	xml_node<>* SAFESTEP_xml = T->first_node("SAFESTEP");
	SAFESTEP = double(atof(SAFESTEP_xml->value()));
	xml_node<>* PLAN_PERIOD_xml = T->first_node("PLAN_PERIOD");
	PLAN_PERIOD = double(atof(PLAN_PERIOD_xml->value()));
	xml_node<>* STATIC_SAFE_LENGTH_xml = T->first_node("STATIC_SAFE_LENGTH");
	STATIC_SAFE_LENGTH = double(atof(STATIC_SAFE_LENGTH_xml->value()));
	xml_node<>* SAFE_LENGTH_MARGIN_xml = T->first_node("SAFE_LENGTH_MARGIN");
	SAFE_LENGTH_MARGIN = double(atof(SAFE_LENGTH_MARGIN_xml->value()));
	xml_node<>* POINTS_NUM_xml = T->first_node("POINTS_NUM");
	POINTS_NUM = atoi(POINTS_NUM_xml->value());

	DL = L - L_F_BA;
}

vector<double> _solve_cubic(const double &a, const double &b, const double &c, const double &d)
{
	vector<double> result;

	if (a == 0)
	{
		if (b == 0)
		{
			if (c == 0)
				return result;
			else
				result.push_back(-d / c);
		}
		else
		{
			double delta = pow(c, 2) - 4 * b*d;
			if (delta > 0)
			{
				double x1 = (-c + sqrt(delta)) / (2 * b);
				double x2 = (-c - sqrt(delta)) / (2 * b);
				result.push_back(x1);		result.push_back(x2);
				return result;
			}
			else if (delta == 0)
			{
				result.push_back(-c / (2 * b));
				return result;
			}
		}
	}

	double A = pow(b, 2) - 3 * a*c;
	double B = b*c - 9 * a*d;
	double C = pow(c, 2) - 3 * b*d;
	double delta = pow(B, 2) - 4 * A*C;

	if (A == 0 && B == 0)
	{
		result.push_back(-b / (3 * a));
		return result;
	}

	if (delta > 0)
	{
		//X1=(£­b£­(Y1)^(1/3)£­(Y2)^(1/3))/(3a)£»
		//Y1£¬Y2=Ab+3a(£­B¡À(B^2£­4AC)^(1/2))/2
		double Y1 = A*b + 3 * a*(-B + sqrt(delta)) / 2;
		double Y2 = A*b + 3 * a*(-B - sqrt(delta)) / 2;

		double x = (-b - pow(Y1, 1 / 3) - pow(Y2, 1 / 3)) / (3 * a);
		result.push_back(x);
		return result;
	}
	else if (delta == 0)
	{
		double K = B / A;
		double x1 = -b / a + K;
		double x2 = -K / 2;

		if (x1 < x2)
		{
			result.push_back(x1);		result.push_back(x2);
		}
		else
		{
			result.push_back(x2);		result.push_back(x1);
		}
		return result;
	}
	else
	{
		double T = (2 * A*b - 3 * a*B) / (2 * pow(A, 1.5));
		if (std::abs(T) > 1)
			return result;
		double theta = acos(T);

		double x1 = (-b - 2 * sqrt(A)*cos(theta / 3)) / (3 * a);
		double x2 = (-b + sqrt(A)*(cos(theta / 3) + sqrt(3)*sin(theta / 3))) / (3 * a);
		double x3 = (-b + sqrt(A)*(cos(theta / 3) - sqrt(3)*sin(theta / 3))) / (3 * a);

		result.push_back(x1);		result.push_back(x2);		result.push_back(x3);
		std::sort(result.begin(), result.end());
		return result;
	}
}
