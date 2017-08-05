#ifndef SHOW_H
#define SHOW_H

//������opencv���
#include <fstream>
#include <string>
#include <windows.h>
using std::ofstream;

#include <opencv.hpp>

#include "Environment.h"

void clear();

void clearImage(IplImage* src, const CvScalar &color);

void showCoordinate(IplImage* src, const CvPoint &origin, const CvScalar &color, const int &thickness);
void showGrid(IplImage* src, const CvPoint &origin, const int &grid, const CvScalar &color, const int &thickness);

void showLine(IplImage* src, const CvPoint &origin, const vector<double>& vecx, const vector<double>& vecy, const CvScalar &color, const int &thickness);
void showLine(IplImage* src, const CvPoint &origin, const vector<double>& vecx, const vector<double>& vecy, const int &startIndex, const int &endIndex, const CvScalar &color, const int &thickness);

// ��ʾ������xyΪ�����������꣬thetaΪ����ǣ���������veh={l,w,lfba}ΪĬ���������
void showVehicle(IplImage* src, const CvPoint &origin, const double &x, const double &y, const double &theta, const CvScalar &color, const int &thickness, const vector<double> veh = { L, W, L_F_BA });
void showVehicle(IplImage* src, const CvPoint &origin, const vector<double>& vecx, const vector<double>& vecy, const CvScalar &color, const int &thickness, const vector<double> veh = { L, W, L_F_BA });
void showVehicle(IplImage* src, const CvPoint &origin, const vector<double>& vecx, const vector<double>& vecy, const int &startIndex, const int &endIndex, const CvScalar &color, const int &thickness, const vector<double> veh = { L, W, L_F_BA });

// ��ʾ������ͼ
void showEnvir(IplImage* src, const CvPoint &origin, const Environment::time_EnvironMap &envir, const CvScalar &color);

void _environment(Environment::time_EnvironMap::time_en_map *environment);
void _collimap();
/*void _controlpoints(const vector<Point2D> &controlpoints);
void _pathnodes(const RRT::RTG_RRT &tree);
void _treenodes(const RRT::RTG_RRT &tree);
void _route_tree(const RRT::RTG_RRT &tree);*/

#endif