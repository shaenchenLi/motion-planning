#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <algorithm>
#include <vector>
using std::vector;

#include "../include/tinyspline.h"
#include "../include/tinysplinecpp.h"
#include "../include/rapidxml.hpp"
#include "../include/rapidxml_iterators.hpp"
#include "../include/rapidxml_print.hpp"
#include "../include/rapidxml_utils.hpp"

using namespace rapidxml;

#define PI 3.141592653589793
#define TWO_PI 6.283185307179586

#define TURN_RAND_STEP1 60  //12
#define TURN_RAND_STEP2 200  //40
#define U_TURN_RAND_STEP1 20  //4
#define U_TURN_RAND_STEP2 120  //24
#define U_TURN_RAND_STEP3 220  //44

/*******************Environment Parameters getting from Environment Map******************/
#define XMAX 15.//71.
#define XMIN -5.
#define YMAX 15.
#define YMIN -5.//0.
#define TMAX 5.
//#define XMAX 100.//71.
//#define XMIN -5.
//#define YMAX 1.75
//#define YMIN -5.25//0.
//#define TMAX 1.
//#define XMAX 100.//71.
//#define XMIN -5.
//#define YMAX 5.25
//#define YMIN -1.75//0.
//#define TMAX 1.
//#define XMAX 71.
//#define XMIN 0.
//#define YMAX 43.5
//#define YMIN 0.
#define INTERVAL 0.5
#define T_INTERVAL 0.05
#define LANE_WIDTH 3.5 

#define S_SEGMENT 100 //在不规划时默认输出100个0
#define KMAX_real 0.26

extern double RRTSTEP_s, RRTSTEP_l;
extern int MAXITER_1, MAXITER_2;
extern int kNN_NUM;
extern int BT_MAX;
extern int MAX_TRAJ_ITER;

extern double L, W, LA, L_F_BA, KMAX, VMAX, AMIN, AMAX, ALATER, DFAIMAX, ALFA_MIN, V_PRE, DL;

extern double ERROR_SAFE, ERROR_OBS, ERROR_KMAX, ERROR_V, ERROR_BRAKE, ERROR_T;

extern double SAFESTEP, POINTS_NUM, PLAN_PERIOD, STATIC_SAFE_LENGTH, SAFE_LENGTH_MARGIN;

void read_config();

vector<double> _solve_cubic(const double &a, const double &b, const double &c, const double &d);

#endif