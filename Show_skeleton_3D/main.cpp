#include"include/stdafx.h"
#include"include/common.h"
#include"include/visualizer.h"


int main()
{
	RNG rng(0xFFFFFFFF);
	char filenamecolor[500],filenameDepth[500], filenameKeypoint[500],filenameimagewrite[500];
	int numberdata = 14;
	int framecount=59;
	int framestart= 59;
	int numberframe = 126;

	cv::Mat color_image,depth_map;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_objects (new pcl::visualization::PCLVisualizer ("3D Viewer Objects on The Table"));

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_scene(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr vec_cloud_keypoint(new pcl::PointCloud<pcl::PointXYZ>);
	while (framecount < numberframe)
	{

		std::cout << "Frame number: " << framecount << std::endl;
		//sprintf(filenamecolor, "G:/PROJECT_CODE/dulieu_ALL/Du_lieu_color_depth/l3/color/FrameC_%d.png", framecount);
		sprintf(filenamecolor, "G:/PROJECT_CODE/cocoapi/Du_lieu2_14/poses/color_l%d/FrameC_%d.png", numberdata,framecount);
	
		color_image = cv::imread(filenamecolor, cv::IMREAD_COLOR);

		sprintf(filenameDepth, "G:/PROJECT_CODE/dulieu_ALL/Du_lieu_color_depth/l%d/depth/FrameD_%d.png", numberdata,framecount);
		depth_map = cv::imread(filenameDepth, cv::IMREAD_UNCHANGED);
		depth_map.convertTo(depth_map, CV_32F);
		if (depth_map.rows > 0 && color_image.rows > 0)
		{

			sprintf(filenameKeypoint, "G:/PROJECT_CODE/dulieu_ALL/Du_lieu_color_depth/l%d/data_edit/FrameC_%d.txt", numberdata,framecount);
			getcoorAccelerationVeector getac;
			std::vector<Vector4> vec_keypoint;
			getac.getKeypoint(filenameKeypoint, vec_keypoint);
			
			/////////////////////////////////////////////////////////////////////
			double start, end, elapsed_secs;
			start = clock();

			///////////////////////////////////////////////////////////////////////
			//down sampling
			createXYZRGBPointCloud(depth_map, color_image, cloud_scene);


			/*pcl::visualization::CloudViewer viewer_cup("cup1");
			viewer_cup.showCloud(cloud_scene);
			while (!viewer_cup.wasStopped())
			{
			}*/

			//std::vector<pcl::PointXYZRGB> vec_cloud_keypoint;
			
			createXYZRGBPointCloud_keypoint(depth_map, color_image, vec_keypoint, vec_cloud_keypoint);

			visualization_scene view;
			//view.visuali_two(viewer_objects, cloud_scene, vec_cloud_keypoint, framecount);
			view.visuali_pointcloud_keypoint(viewer_objects, cloud_scene, vec_cloud_keypoint, framecount, numberdata, framestart);
			//create Key points
		}
		framecount = framecount + 1;
		cloud_scene->clear();
		vec_cloud_keypoint->clear();
	}

	return 0;
}