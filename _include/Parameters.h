#ifndef CONFIGURATION_H
#define CONFIGURATION_H

/**************************************RRT Parameters*************************************/
#define RRTSTEP_s 2. //最好是SAFESTEP的整数倍，方便计算后插入
#define RRTSTEP_l 8. //两种不同的RRTSTEP可以选择以提高效率
#define MAXITER 1000
#define SAFESTEP 2. //max(L_F_BA, interval of environment)
#define kNN_NUM 5 //kNN个数
#define BT_MAX 10 //最大回溯次数

/************************************Vehicle Parameters***********************************/
#define PI 3.1415926535898
#define TWO_PI (2*PI)
#define L 3.569
#define W 1.551
#define H 1.5
#define LA 2.305
#define DL 0.5
#define L_F_BA 3.069
#define KMAX 0.26
#define KMIN (-0.26)
#define ALATER 0.4*9.8
#define VMAX 20.
#define WMAX 1.56 //rad/s

/*******************Environment Parameters getting from Environment Map******************/
#define XMAX 33.
#define XMIN 0.0
#define YMAX 30.
#define YMIN -22.75
#define INTERVAL 0.15

/********************Pre_traj Parameters getting from Environment Map*******************/
#define ERROR_1 0.3 //距离约束的距离 需要大于地图间隔
#define ERROR_2 0.05 //确保曲率的距离
#define ERROR_3 0.2
#define ERROR_4 1.

/******************************Database relative parameters******************************/
#define THETA_max_L (PI/4)
#define THETA_min_L (-PI/4)
#define THETA_min_LT (0.25*PI)
#define THETA_max_LT (0.75*PI)
#define THETA_min_U (0.75*PI)
#define THETA_max_U (1.25*PI)

/****************************************************************************************/
struct Point2D
{
	Point2D()
	{
		x = y = 0;
	}
	Point2D(const double &x0, const double &y0) :x(x0), y(y0) {}
	Point2D(const Point2D &p) : x(p.x), y(p.y) {}

	void reset(double x0, double y0)
	{
		x = x0;
		y = y0;
	}

	double x, y;
};

#endif