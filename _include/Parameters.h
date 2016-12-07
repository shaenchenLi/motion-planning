#ifndef CONFIGURATION_H
#define CONFIGURATION_H

/**************************************RRT Parameters*************************************/
#define RRTSTEP_s 2.f //最好是SAFESTEP的整数倍，方便计算后插入
#define RRTSTEP_l 8.f //两种不同的RRTSTEP可以选择以提高效率
#define MAXITER 1000
#define SAFESTEP 2.f //max(L_F_BA, interval of environment)

/************************************Vehicle Parameters***********************************/
#define PI 3.1415926535898f
#define TWO_PI (2*PI)
#define L 3.569f
#define W 1.551f
#define H 1.5f
#define LA 2.305f
#define DL 0.5f
#define L_F_BA 3.069f
#define KMAX 0.26f
#define KMIN (-0.26f)
#define ALATER 0.4f*9.8f
#define VMAX 20.f
#define WMAX 1.56f //rad/s

/*******************Environment Parameters getting from Environment Map******************/
#define XMAX 33.f
#define XMIN 0.0f
#define YMAX 30.f
#define YMIN -22.75f
#define INTERVAL 0.15f

/********************Pre_traj Parameters getting from Environment Map*******************/
#define ERROR_1 0.3f //距离约束的距离 需要大于地图间隔
#define ERROR_2 0.05f //确保曲率的距离
#define ERROR_3 0.2f
#define ERROR_4 1.f

/******************************Database relative parameters******************************/
#define THETA_max_L (PI/4)
#define THETA_min_L (-PI/4)
#define THETA_min_LT (0.25f*PI)
#define THETA_max_LT (0.75f*PI)
#define THETA_min_U (0.75f*PI)
#define THETA_max_U (1.25f*PI)

/****************************************************************************************/
struct Point2D
{
	Point2D()
	{
		x = y = 0;
	}
	Point2D(const float &x0, const float &y0) :x(x0), y(y0) {}
	Point2D(const Point2D &p) : x(p.x), y(p.y) {}

	void reset(float x0, float y0)
	{
		x = x0;
		y = y0;
	}

	float x, y;
};

#endif