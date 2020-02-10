#ifndef VISUALIZER
#define VISUALIZER

#include"stdafx.h"
//#include"Estimate_normal.h"
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <iostream>
//#include <boost/thread/thread.hpp>
//#include <pcl/common/common_headers.h>
//#include <pcl/console/parse.h>

struct color_value
{
	int r;
	int g;
	int b;
};


using namespace std;
class visualization_scene
{
	
public:

	void visualiall( boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene,  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters, std::vector<int> task, int frame);
	void visuali(char filewrite[200],boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,int frame,int startframe);

	void visuali_one(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int frame);

	void visuali_two(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, 
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1 ,int frame);
	
	void visuali_two_color(int startfram,int mohinh, int dataso, boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudobject,std::vector<pcl::ModelCoefficients::Ptr> vect_coefficitentmodel, int frame);

	void visuali_two_color_vectorcloud(char fileimagewrite[500], boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, 
		pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_scene, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> vector_cloudobject,
		std::vector<int> &vector_typeoject, std::vector<pcl::ModelCoefficients::Ptr> &Vector_ceoficientobject, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr tableplane, pcl::ModelCoefficients::Ptr &ceoplane,
		int frame,int startframe);


	
	void visuali_three(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2,int frame);
	void visualiall_multiplane(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene,std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters, int frame);
	


	void visualiall_multiplanecolor(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data_r,
	pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals,std::vector<int> sampleselect,
	pcl::ModelCoefficients &cylinder_coefficitent,int type_model,int numdata,int frame,int iter_run, char filename[1000],char filenamer[1000], char filenamecylinder[1000]);



	void visualiall_multiplane_Normal(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters);
	void visuali_two_color_only(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,int frame);
	void visuali_cylinder(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normal, pcl::ModelCoefficients &cylinder_coefficitent,int frame);
	
	void visuali_cylinderxyz(int method_d,int model_type,boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1,
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_r, pcl::ModelCoefficients &cylinder_coefficitent,int frame);


	void visuali_pointcloud_keypoint( boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_scene,
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_keypoint, int frame, int numberdata, int framestart);
};
#endif