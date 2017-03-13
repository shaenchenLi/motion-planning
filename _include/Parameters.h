#ifndef CONFIGURATION_H
#define CONFIGURATION_H

/**************************************RRT Parameters*************************************/
#define RRTSTEP_s 2. //�����SAFESTEP���������������������
#define RRTSTEP_l 8. //���ֲ�ͬ��RRTSTEP����ѡ�������Ч��
#define MAXITER_1 1000  //200
#define MAXITER_2 200
#define SAFESTEP 2. //max(L_F_BA, interval of environment)
#define kNN_NUM 10 //kNN����
#define BT_MAX 10 //�����ݴ���
#define TURN_RAND_STEP1 60  //12
#define TURN_RAND_STEP2 200  //40
#define U_TURN_RAND_STEP1 20  //4
#define U_TURN_RAND_STEP2 120  //24
#define U_TURN_RAND_STEP3 220  //44

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
#define ALATER (0.4*9.8)
#define VMAX 20.
#define WMAX 1.56 //rad/s
#define AMAX (0.9*9.8)
#define AMIN (-0.9*9.8)
#define DFAIMAX (PI)
#define ERROR_V 0.5

/*******************Environment Parameters getting from Environment Map******************/
#define XMAX 71.
#define XMIN 0.
#define YMAX 43.5
#define YMIN 0.
#define INTERVAL 0.15
#define LANE_WIDTH 3.5 

/********************Pre_traj Parameters getting from Environment Map*******************/
#define ERROR_1 0.3 //����Լ���ľ��� ��Ҫ���ڵ�ͼ���
#define ERROR_2 0.05 //ȷ�����ʵľ���
#define ERROR_3 0.2
#define ERROR_4 1.

/****************Parameters used to select fitpoints for curvature curve****************/
#define DELTA_K 0.005 //���������ʼ�����ڸ�ֵʱ��¼һ��k
#define SHARP_K 0.5 //����delta_k֮����������Ҫ��¼��������Ϊ��k��һ�����
#define DU 0.001 //����B��������ʱ���u�Ĵ�С

/****************************************************************************************/
struct Point2D
{
	Point2D()
	{
		x = y = 0.;
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