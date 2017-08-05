#include "show.h"

ofstream outfile;
std::string base_file = "validate\\";
char filename[100];

int WIN_X = (int)std::ceil(XMAX / INTERVAL);
int WIN_Y = (int)std::ceil(YMAX / INTERVAL);
int TEXT_H = (int)std::ceil(WIN_X / 3);
int WIN_SCALE = std::max(WIN_X, WIN_Y) > 300 ? 1 : 2;

void _environment(Environment::time_EnvironMap::time_en_map *environment)
{
	std::string doc = "environment.txt";
	//system("cd ../validate");
	sprintf_s(filename, sizeof(filename), "%s%s", base_file.c_str(), doc.c_str());
	outfile.open(filename, std::ios::trunc);
	for (auto &e : *environment)
		outfile << e.first.x << " " << e.first.y << " "  << e.first.t << " " << e.second << endl;
	outfile.close();
}

void clear()
{
	//system("cd ../validate");
	std::string doc = "ctrlpoints.txt";
	sprintf_s(filename, sizeof(filename), "%s%s", base_file.c_str(), doc.c_str());
	outfile.open(filename, std::ios::trunc);
	outfile.close();
	doc = "treenode.txt";
	sprintf_s(filename, sizeof(filename), "%s%s", base_file.c_str(), doc.c_str());
	outfile.open(filename, std::ios::trunc);
	outfile.close();
	doc = "path.txt";
	sprintf_s(filename, sizeof(filename), "%s%s", base_file.c_str(), doc.c_str());
	outfile.open(filename, std::ios::trunc);
	outfile.close();
}

void clearImage(IplImage* src, const CvScalar &color)
{
	for (int y = 0; y < src->height; y++)
	{
		for (int x = 0; x < src->width; x++)
		{
			double* temp_ptr_src = &((double*)(src->imageData + src->widthStep*y))[x * 3];
			temp_ptr_src[0] = color.val[0];
			temp_ptr_src[1] = color.val[1];
			temp_ptr_src[2] = color.val[2];
		}
	}
}

void showCoordinate(IplImage* src, const CvPoint &origin, const CvScalar &color, const int &thickness)
{
	cvLine(src, origin, cvPoint(WIN_Y*WIN_SCALE, TEXT_H*WIN_SCALE), color, thickness);
	cvLine(src, origin, cvPoint(0, (WIN_X + TEXT_H)*WIN_SCALE), color, thickness);
}

void showGrid(IplImage* src, const CvPoint &origin, const int &grid, const CvScalar &color, const int &thickness)
{
	//画图x轴，opencv与y轴平行的线
	for (int y = grid; y < WIN_Y*WIN_SCALE; y += grid)
		cvLine(src, cvPoint(origin.x + y, origin.y), cvPoint(origin.x + y, origin.y + WIN_Y*WIN_SCALE), color, thickness);

	//画图y轴，opencv与x轴平行的线
	for (int x = grid; x < WIN_X*WIN_SCALE; x += grid)
		cvLine(src, cvPoint(origin.x, origin.y + x), cvPoint(origin.x + (WIN_X + TEXT_H)*WIN_SCALE, origin.y + x), color, thickness);
}

void showLine(IplImage* src, const CvPoint &origin, const vector<double>& vecx, const vector<double>& vecy, const CvScalar &color, const int &thickness)
{
	int lenx = (int)vecx.size();
	int leny = (int)vecy.size();

	if (lenx && lenx == leny)
	{
		for (int i = 1; i < lenx; i++)
		{
			CvPoint line_start = cvPoint(WIN_SCALE*(int)std::floor((vecy[i - 1] - YMIN) / INTERVAL) + origin.x, WIN_SCALE*(int)std::floor((vecx[i - 1] - XMIN) / INTERVAL) + origin.y);
			CvPoint line_end = cvPoint(WIN_SCALE*(int)std::floor((vecy[i] - YMIN) / INTERVAL) + origin.x, WIN_SCALE*(int)std::floor((vecx[i] - XMIN) / INTERVAL) + origin.y);
			cvLine(src, line_start, line_end, color, thickness);
		}
	}
	else
		MessageBox(nullptr, "Lines Show Error!", "ERROR", MB_OKCANCEL | MB_ICONERROR);
}

void showLine(IplImage* src, const CvPoint &origin, const vector<double>& vecx, const vector<double>& vecy, const int &startIndex, const int &endIndex, const CvScalar &color, const int &thickness)
{
	int lenx = (int)vecx.size();
	int leny = (int)vecy.size();
	if (lenx && lenx == leny)
	{
		for (int i = startIndex; i < endIndex + 1; i++)
		{
			CvPoint line_start = cvPoint(WIN_SCALE*(int)std::floor((vecy[i - 1] - YMIN) / INTERVAL) + origin.x, WIN_SCALE*(int)std::floor((vecx[i - 1] - XMIN) / INTERVAL) + origin.y);
			CvPoint line_end = cvPoint(WIN_SCALE*(int)std::floor((vecy[i] - YMIN) / INTERVAL) + origin.x, WIN_SCALE*(int)std::floor((vecx[i] - XMIN) / INTERVAL) + origin.y);
			cvLine(src, line_start, line_end, color, thickness);
		}
	}
	else
		MessageBox(nullptr, "Lines Show Error!", "ERROR", MB_OKCANCEL | MB_ICONERROR);
}

//veh={l,w,lfba}
void showVehicle(IplImage* src, const CvPoint &origin, const double &x, const double &y, const double &theta, const CvScalar &color, const int &thickness, const vector<double> veh)
{
	double l = veh[0], w = veh[1], lfba = veh[2];
	vector<double> veh_x, veh_y;
	veh_x.push_back(x - 0.5*w*sin(theta) - (l - lfba)*cos(theta));		veh_y.push_back(y + 0.5*w*cos(theta) - (l - lfba)*sin(theta));
	veh_x.push_back(x - 0.5*w*sin(theta) + lfba*cos(theta));			veh_y.push_back(y + 0.5*w*cos(theta) + lfba*sin(theta));
	veh_x.push_back(x + 0.5*w*sin(theta) + lfba*cos(theta));			veh_y.push_back(y - 0.5*w*cos(theta) + lfba*sin(theta));
	veh_x.push_back(x + 0.5*w*sin(theta) - (l - lfba)*cos(theta));		veh_y.push_back(y - 0.5*w*cos(theta) - (l - lfba)*sin(theta));
	//重复画第一个点，从而形成闭合矩形
	veh_x.push_back(x - 0.5*w*sin(theta) - (l - lfba)*cos(theta));		veh_y.push_back(y + 0.5*w*cos(theta) - (l - lfba)*sin(theta));

	showLine(src, origin, veh_x, veh_y, color, thickness);
}

void showVehicle(IplImage* src, const CvPoint &origin, const vector<double>& vecx, const vector<double>& vecy, const CvScalar &color, const int &thickness, const vector<double> veh)
{
	int lenx = (int)vecx.size();
	int leny = (int)vecy.size();
	if (lenx && lenx == leny)
	{
		vector<double> vectheta(lenx);
		for (int i = 0; i < lenx - 1; i++)
			vectheta[i] = atan2(vecy[i + 1] - vecy[i], vecx[i + 1] - vecx[i]);
		vectheta[lenx - 1] = vectheta[lenx - 2];

		for (int i = 0; i < lenx; i++)
			showVehicle(src, origin, vecx[i], vecy[i], vectheta[i], color, thickness, veh);
	}
	else
		MessageBox(nullptr, "Lines Show Error!", "ERROR", MB_OKCANCEL | MB_ICONERROR);
}

void showVehicle(IplImage* src, const CvPoint &origin, const vector<double>& vecx, const vector<double>& vecy, const int &startIndex, const int &endIndex, const CvScalar &color, const int &thickness, const vector<double> veh)
{
	int lenx = (int)vecx.size();
	int leny = (int)vecy.size();
	if (lenx && lenx == leny)
	{
		vector<double> vectheta(lenx);
		for (int i = 0; i < lenx - 1; i++)
			vectheta[i] = atan2(vecy[i + 1] - vecy[i], vecx[i + 1] - vecx[i]);
		vectheta[lenx - 1] = vectheta[lenx - 2];

		for (int i = startIndex; i < endIndex + 1; i++)
			showVehicle(src, origin, vecx[i], vecy[i], vectheta[i], color, thickness, veh);
	}
	else
		MessageBox(nullptr, "Lines Show Error!", "ERROR", MB_OKCANCEL | MB_ICONERROR);
}

void showEnvir(IplImage* src, const CvPoint &origin, Environment::time_EnvironMap *environmap, const CvScalar &obs_color, const CvScalar &free_color)
{
	Environment::time_EnvironMap::time_en_map* envir = environmap->_environment();
	double xmin = environmap->_range()->begin()->x;
	double ymin = environmap->_range()->begin()->y;
	double xmax = environmap->_range()->rbegin()->x;
	double ymax = environmap->_range()->rbegin()->y;
	double inter = environmap->_interval();
			
	int N_t = (int)std::ceil(environmap->_t_range() / environmap->_time_interval());

	//需要一个偏移量
	xmin = std::max(xmin, XMIN);
	ymin = std::max(ymin, YMIN);
	xmax = std::min(xmax, XMAX);
	ymax = std::max(ymax, YMAX);
	for (double x = xmin; x < xmax; x += inter)
	{
		for (double y = ymin; y < ymax; y += inter)
		{
			int x0 = (int)std::floor((x - xmin) / inter);
			int y0 = (int)std::floor((y - ymin) / inter);

			CvPoint point = cvPoint(WIN_SCALE*(int)std::floor((y - ymin) / INTERVAL) + origin.x, WIN_SCALE*(int)std::floor((x - xmin) / INTERVAL) + origin.y);
			
			int is_safe = 0;
			for (int t = 0; t <= N_t; t++)
			{
				if (envir->at({ x0, y0, t }) != 0)
				{
					is_safe = 1;
					break;
				}
			}

			if (is_safe == 1)
				cvCircle(src, point, 2, cv::Scalar(obs_color));
			else
				cvCircle(src, point, 2, cv::Scalar(free_color));
		}
	}
}