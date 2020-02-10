#ifndef GET_COOR_ACC
#define GET_COOR_ACC
#include <iostream>  // for cout, getline
#include <sstream>  // for istringstream
#include <string>   // for string
#include <opencv2\highgui\highgui.hpp>
#include <fstream>
#include"stdafx.h"
using namespace cv;
using namespace std;
class getcoorAccelerationVeector
{
public:
	void getcooracc(char pathInput[1000],int row, int maxrow, double matrix[4]);
	void getcooracc_allrow(char pathInput[1000],int row, int maxrow, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_target3point);
	void getKeypoint(char pathInput[1000], std::vector<Vector4> &vec_keypoint);
};
#endif