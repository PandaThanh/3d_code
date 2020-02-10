/** @file common.cpp
 *  @brief Common function definitions for the different projects.
 *
 *
 *  @author Michiel Vlaminck
 *
 *	@date Nov 3, 2015
 *  
 *	@bug n/a
 */

#include "include/stdafx.h"
#include "include/common.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include"opencv2/core/core.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <pcl/surface/convex_hull.h>

void createDepthMap(cv::Mat &rawDepthMap, cv::Mat &depthMap)
{
	for(int y = 0; y < rawDepthMap.rows; y++) {
		for(int x = 0; x < rawDepthMap.cols; x++) {
			depthMap.at<float>(y,x) = (256*rawDepthMap.at<cv::Vec3b>(y,x)[0] + rawDepthMap.at<cv::Vec3b>(y,x)[1])/8;
			depthMap.at<float>(y,x) = depthMap.at<float>(y,x)/1000;
		}
	}
}

void createDepthMap(cv::Mat &depthMap) {

	for(int y = 0; y < depthMap.rows; y++) {
		for(int x = 0; x < depthMap.cols; x++) {
			depthMap.at<float>(y,x) = depthMap.at<float>(y,x)/1000;
		}
	}

}
int round_int(double r) {
	return (r > 0.0) ? (r + 0.5) : (r - 0.5);
}
void createDepthImagefromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, cv::Mat& depthImage, cv::Rect &Object_Rect)
{
	NUI_IMAGE_RESOLUTION CAMERA_RESOLUTION = NUI_IMAGE_RESOLUTION_640x480;
	int min_col = INT16_MAX;
	int min_row = INT16_MAX;
	int max_col = 0;
	int max_row = 0;

			for (int i = 0; i < input_cloud->points.size(); i++)
			{
				Vector4 vector4;
				LONG *plDepthX;
				plDepthX = 0;
				LONG *plDepthY;
				plDepthY = 0;
				USHORT *pusDepthValue;
				pusDepthValue = 0;
				vector4.x = input_cloud->points[i].x;
				vector4.y = input_cloud->points[i].y;
				vector4.z = input_cloud->points[i].z;

				

				FLOAT depthX1 = 0, depthY1 = 0;
				::NuiTransformSkeletonToDepthImage(vector4, &depthX1, &depthY1, CAMERA_RESOLUTION);
				
				int depthX = round_int(depthX1), depthY = round_int(depthY1);

				if (min_col > (int)depthX)
				{
					min_col = (int)depthX;
				}
				if (min_row > (int)depthY)
				{
					min_row = (int)depthY;
				}
				if (max_col < (int)depthX)
				{
					max_col = (int)depthX;
				}
				if (max_row < (int)depthY)
				{
					max_row = (int)depthY;
				}
			}


			//cv::Mat cropedImage = depthImage(cv::Rect(min_col, min_row, max_col - min_col, max_row - min_row));
			Object_Rect= cv::Rect(min_col, min_row, max_col - min_col, max_row - min_row);
			//cv::imshow("Display window", cropedImage);                // Show our image inside it.
			//cv::waitKey(0); // Wait for a keystroke in the window
}
void createXYZRGBPointCloud1(cv::Mat& depthImage, cv::Mat& rgbImage, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud)
{
	pcl::PointXYZRGB newPoint;
	//double maxdepth=0;
	for (int i=0;i<depthImage.rows;i++)
	{
		for (int j=0;j<depthImage.cols;j++)
		{
			float depthValue = depthImage.at<float>(i,j);

			USHORT handDist = (USHORT)depthValue;
			long depthX = (long)j;
			long depthY = (long)i;
			Vector4 vector4 = NuiTransformDepthImageToSkeleton(depthX, depthY, handDist,NUI_IMAGE_RESOLUTION_640x480); 

			/*if(maxdepth<depthValue)
			{
			maxdepth=depthValue;*/
			if(depthValue > 0)              // if depthValue is not NaN
			{
				// Find 3D position respect to rgb frame:
				newPoint.z = vector4.z;//depthValue;
				newPoint.x = vector4.x;//(j - rgb_intrinsic_matrix(0,2)) * newPoint.z * rgb_focal_inverted_x;
				newPoint.y = vector4.y;//(i - rgb_intrinsic_matrix(1,2)) * newPoint.z * rgb_focal_inverted_y;
				newPoint.r = rgbImage.at<cv::Vec3b>(i,j)[2];
				newPoint.g = rgbImage.at<cv::Vec3b>(i,j)[1];
				newPoint.b = rgbImage.at<cv::Vec3b>(i,j)[0];
				output_cloud->push_back(newPoint);
			}
			else
			{
				newPoint.z = std::numeric_limits<float>::quiet_NaN();
				newPoint.x = std::numeric_limits<float>::quiet_NaN();
				newPoint.y = std::numeric_limits<float>::quiet_NaN();
				newPoint.r = std::numeric_limits<unsigned char>::quiet_NaN();
				newPoint.g = std::numeric_limits<unsigned char>::quiet_NaN();
				newPoint.b = std::numeric_limits<unsigned char>::quiet_NaN();
				output_cloud->push_back(newPoint);
			}
			//}
		}
	}

	output_cloud->height = depthImage.rows;
	output_cloud->width = depthImage.cols;
	output_cloud->is_dense = false;
	output_cloud->points.resize (output_cloud->width * output_cloud->height);
}
void createXYZRGBPointCloud2(cv::Mat& depthImage, cv::Mat& rgbImage, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud)
{
	pcl::PointXYZRGB newPoint;
	//double maxdepth=0;
	for (int i=0;i<depthImage.rows;i++)
	{
		for (int j=0;j<depthImage.cols;j++)
		{
			float depthValue = depthImage.at<float>(i,j);

			USHORT handDist = (USHORT)depthValue;
			long depthX = (long)j;
			long depthY = (long)i;
			Vector4 vector4 = NuiTransformDepthImageToSkeleton(depthX, depthY, handDist,NUI_IMAGE_RESOLUTION_640x480); 

			/*if(maxdepth<depthValue)
			{
			maxdepth=depthValue;*/
			if(depthValue > 0 && depthValue<1600)              // if depthValue is not NaN
			{
				// Find 3D position respect to rgb frame:
				newPoint.z = vector4.z;//depthValue;
				newPoint.x = vector4.x;//(j - rgb_intrinsic_matrix(0,2)) * newPoint.z * rgb_focal_inverted_x;
				newPoint.y = vector4.y;//(i - rgb_intrinsic_matrix(1,2)) * newPoint.z * rgb_focal_inverted_y;
				newPoint.r = rgbImage.at<cv::Vec3b>(i,j)[2];
				newPoint.g = rgbImage.at<cv::Vec3b>(i,j)[1];
				newPoint.b = rgbImage.at<cv::Vec3b>(i,j)[0];
				output_cloud->push_back(newPoint);
			}
			else
			{
				newPoint.z = std::numeric_limits<float>::quiet_NaN();
				newPoint.x = std::numeric_limits<float>::quiet_NaN();
				newPoint.y = std::numeric_limits<float>::quiet_NaN();
				newPoint.r = std::numeric_limits<unsigned char>::quiet_NaN();
				newPoint.g = std::numeric_limits<unsigned char>::quiet_NaN();
				newPoint.b = std::numeric_limits<unsigned char>::quiet_NaN();
				output_cloud->push_back(newPoint);
			}
			//}
		}
	}

	output_cloud->height = depthImage.rows;
	output_cloud->width = depthImage.cols;
	output_cloud->is_dense = false;
	output_cloud->points.resize (output_cloud->width * output_cloud->height);
}
void createXYZRGBPointCloud_superpixel(cv::Mat& depthImage, cv::Mat& rgbImage, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud)
{
	// =======================================
	//		Nathan Silberman parameters
	// =======================================
	Eigen::Matrix3f rgb_intrinsic_matrix;

	rgb_intrinsic_matrix(0,0) = 518.85790117450188f; //fx
	rgb_intrinsic_matrix(1,1) = 519.46961112127485f; //fy
	rgb_intrinsic_matrix(0,2) = 325.58244941119034f; //cx
	rgb_intrinsic_matrix(1,2) = 253.73616633400465f; //cy

	// ===================================
	//		Nicolas Burrus parameters
	// ===================================
	//rgb_intrinsic_matrix(0,0) = 594.21434211923247f; //fx
	//rgb_intrinsic_matrix(1,1) = 591.04053696870778f; //fy
	//rgb_intrinsic_matrix(0,2) = 339.30780975300314f; //cx
	//rgb_intrinsic_matrix(1,2) = 242.73913761751615f; //cy

	float rgb_focal_inverted_x = 1/rgb_intrinsic_matrix(0,0);	// 1/fx
	float rgb_focal_inverted_y = 1/rgb_intrinsic_matrix(1,1);	// 1/fy

	pcl::PointXYZRGB newPoint;



	for (int i=0;i<depthImage.rows;i++)
	{
		for (int j=0;j<depthImage.cols;j++)
		{
			if (((i%2)>0) && ((j%2)>0))
			{
				float AveDepth=0;
				int dem=0;
				for(int k=i-1;k<=i+1;k++)
				{
					if(k<480)
					{
					for(int l=j-1;l<=j+1;l++)
					{
						if (l<640)
						{
						float depthValuekl = depthImage.at<float>(k,l);
						if(depthValuekl>0)
							{
							AveDepth=AveDepth + depthValuekl;
							dem=dem+1;
							}
						}
					}
					}
				}
				AveDepth=AveDepth/dem;
				if (AveDepth>0)
				{
					// Find 3D position respect to rgb frame:
					newPoint.z = AveDepth;
					newPoint.x = (j - rgb_intrinsic_matrix(0,2)) * newPoint.z * rgb_focal_inverted_x;
					newPoint.y = (i - rgb_intrinsic_matrix(1,2)) * newPoint.z * rgb_focal_inverted_y;
					newPoint.r = rgbImage.at<cv::Vec3b>(i,j)[2];
					newPoint.g = rgbImage.at<cv::Vec3b>(i,j)[1];
					newPoint.b = rgbImage.at<cv::Vec3b>(i,j)[0];
					output_cloud->push_back(newPoint);
				}
				else
				{
					newPoint.z = std::numeric_limits<float>::quiet_NaN();
					newPoint.x = std::numeric_limits<float>::quiet_NaN();
					newPoint.y = std::numeric_limits<float>::quiet_NaN();
					newPoint.r = std::numeric_limits<unsigned char>::quiet_NaN();
					newPoint.g = std::numeric_limits<unsigned char>::quiet_NaN();
					newPoint.b = std::numeric_limits<unsigned char>::quiet_NaN();
					output_cloud->push_back(newPoint);
				}
			}	
		}
	}
	output_cloud->height = depthImage.rows/2;
	output_cloud->width = depthImage.cols/2;
	output_cloud->is_dense = false;
	output_cloud->points.resize (output_cloud->width * output_cloud->height);
}

void ConvertInverXYZRGBtoRGB(cv::Mat& depthImage, cv::Mat& img, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& Input_cloud)
{
	

	pcl::PointXYZRGB newPoint;
	int dem=0;
	vector<cv::Point> contoursub;
	vector<vector<cv::Point>> contours;


	for (int i=0;i<depthImage.rows;i++)
	{
		for (int j=0;j<depthImage.cols;j++)
		{
			float depthValue = depthImage.at<float>(i,j);

			USHORT handDist = (USHORT)depthValue;
			long depthX = (long)j;
			long depthY = (long)i;
			Vector4 vector4 = NuiTransformDepthImageToSkeleton(depthX, depthY, handDist,NUI_IMAGE_RESOLUTION_640x480); 

			/*if(maxdepth<depthValue)
			{
			maxdepth=depthValue;*/
			if(depthValue > 0 )              // if depthValue is not NaN
			{
				// Find 3D position respect to rgb frame:
				newPoint.z = vector4.z;//depthValue;
				newPoint.x = vector4.x;//(j - rgb_intrinsic_matrix(0,2)) * newPoint.z * rgb_focal_inverted_x;
				newPoint.y = vector4.y;//(i - rgb_intrinsic_matrix(1,2)) * newPoint.z * rgb_focal_inverted_y;

				int co=0;
				for(int p=0;p<Input_cloud->points.size();p++)
				{
					if(Input_cloud->points[p].x==newPoint.x &&Input_cloud->points[p].y==newPoint.y && Input_cloud->points[p].z==newPoint.z)
					{
						co=1;
						break;
					}
				}
				if(co==1)
				{
					dem=dem+1;
					contoursub.push_back(cv::Point(j,i));
					//img.at<uchar>(i,j)=255;
				}
			}
		}
	}

	


	/* cv::imshow("myWindow", im_out); 
	cv::waitKey(0);*/


 	contours.push_back(contoursub);
	vector<vector<cv::Point> >hull( contours.size());
   for( int i = 0; i < contours.size(); i++ )
      {  
		  convexHull(cv::Mat(contours[i]), hull[i], false ); 
	  }
	for( int i = 0; i< hull.size(); i++ )
      {
        cv::Scalar color = cv::Scalar(0, 0, 255);
		drawContours(img, hull, i, color, 5, 8, vector<cv::Vec4i>(), 0, cv::Point(0,0));
		//drawContours( img, hull, i,color, CV_FILLED, 8);
      }

	contours.clear();
	hull.clear();
	contoursub.clear();

	/*cv::imshow("kdfsfdgshf",img);
	cv::waitKey(0);*/


	//// Floodfill from point (0, 0)
 //   cv::Mat im_floodfill = img.clone();
 //   floodFill(im_floodfill, cv::Point(0,0), cv::Scalar(255));
 //    
 //   // Invert floodfilled image
 //   cv::Mat im_floodfill_inv;
 //   bitwise_not(im_floodfill, im_floodfill_inv);
 //    
 //   // Combine the two images to get the foreground.
 //   cv::Mat im_out = (img | im_floodfill_inv);

	//im_floodfill.release();
	//im_floodfill_inv.release();

	//img = im_out.clone();
}




void CreateXYZRGBtoRGB_EachObject(cv::Mat& depthImage_full, cv::Mat& depthImage_anotation, cv::Mat& Color_img,  
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& vec_out_cloudobject)
{
	     
	      for(int k=1;k<7;k++)
		    {
				 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object(new pcl::PointCloud<pcl::PointXYZ>);
				 for(int i=0; i<depthImage_full.rows; ++i) 
					{
					for(int j=0; j<depthImage_full.cols; ++j)
					{
							if (depthImage_anotation.at<uchar>(i, j)==k)
							{
								 pcl::PointXYZ newPoint;
								float depthValue = depthImage_full.at<float>(i,j);

								USHORT handDist = (USHORT)depthValue;
								long depthX = (long)j;
								long depthY = (long)i;
								Vector4 vector4 = NuiTransformDepthImageToSkeleton(depthX, depthY, handDist,NUI_IMAGE_RESOLUTION_640x480); 
								if(depthValue > 0 )              
								{
									newPoint.z = vector4.z;
									newPoint.x = vector4.x;
									newPoint.y = vector4.y;
									/*newPoint.r = Color_img.at<cv::Vec3b>(i,j)[2];
									newPoint.g = Color_img.at<cv::Vec3b>(i,j)[1];
									newPoint.b = Color_img.at<cv::Vec3b>(i,j)[0];*/
									cloud_object->push_back(newPoint);
								}
								else
								{
									/*newPoint.z = std::numeric_limits<float>::quiet_NaN();
									newPoint.x = std::numeric_limits<float>::quiet_NaN();
									newPoint.y = std::numeric_limits<float>::quiet_NaN();*/
									/*newPoint.r = std::numeric_limits<unsigned char>::quiet_NaN();
									newPoint.g = std::numeric_limits<unsigned char>::quiet_NaN();
									newPoint.b = std::numeric_limits<unsigned char>::quiet_NaN();*/
									//cloud_object->push_back(newPoint);
								}
								
							}
					   }
				  }
				 if (cloud_object->size()>0)
				 {
					 vec_out_cloudobject.push_back(cloud_object);
				 }
				 //cloud_object->clear();
		  }
}
void CreateXYZRGBA_Point(cv::Mat& depthImage,cv::Mat& colorImage,int x,int y, pcl::PointCloud<pcl::PointXYZ>::Ptr& A_point)
{
	         for(int i=0; i<depthImage.rows; ++i) 
					{
					for(int j=0; j<depthImage.cols; ++j)
					{
							if ((i==x)&& (j==y))
							{
								pcl::PointXYZ newPoint;
								float depthValue = depthImage.at<float>(i,j);

								USHORT handDist = (USHORT)depthValue;
								long depthX = (long)j;
								long depthY = (long)i;
								Vector4 vector4 = NuiTransformDepthImageToSkeleton(depthX, depthY, handDist,NUI_IMAGE_RESOLUTION_640x480); 
								if(depthValue > 0 )              
								{
									newPoint.z = vector4.z;
									newPoint.x = vector4.x;
									newPoint.y = vector4.y;
									A_point->push_back(newPoint);
								}
								
							}
					}
			 }

}
void CreateXYZRGBTwo_Point(cv::Mat& depthImage,cv::Mat& colorImage,int x1, int y1, int x2, int y2, pcl::PointCloud<pcl::PointXYZ>::Ptr& Two_point)
{
	           for(int i=0; i<depthImage.rows; ++i) 
					{
					for(int j=0; j<depthImage.cols; ++j)
					{
							if (((i==x1)&& (j==y1))|| ((i==x2)&& (j==y2)))
							{
								pcl::PointXYZ newPoint;
								float depthValue = depthImage.at<float>(i,j);

								USHORT handDist = (USHORT)depthValue;
								long depthX = (long)j;
								long depthY = (long)i;
								Vector4 vector4 = NuiTransformDepthImageToSkeleton(depthX, depthY, handDist,NUI_IMAGE_RESOLUTION_640x480); 
								if(depthValue > 0 )              
								{
									newPoint.z = vector4.z;
									newPoint.x = vector4.x;
									newPoint.y = vector4.y;
									Two_point->push_back(newPoint);
								}
								
							}
					}
			 }

}



void createXYZPointCloud_crop(int originx, int originy, int w, int h,cv::Mat& depthImage, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPointCloud)
{

	Eigen::Matrix3f depthIntrinsicMatrix;

	// ===================================
	//		Nathan Silberman parameters
	// ===================================
	depthIntrinsicMatrix(0,0) = 582.62448167737955f; //fx
	depthIntrinsicMatrix(1,1) = 582.69103270988637f; //fy
	depthIntrinsicMatrix(0,2) = 313.04475870804731f; //cx
	depthIntrinsicMatrix(1,2) = 238.44389626620386f; //cy

	float depthFocalInvertedX = 1/depthIntrinsicMatrix(0,0);	// 1/fx
	float depthFocalInvertedY = 1/depthIntrinsicMatrix(1,1);	// 1/fy

	pcl::PointXYZ newPoint;

	for (int i=originy;i<originy+h;i++)
	{
		for (int j=originx;j<originx+w;j++)
		{
			float depthValue = depthImage.at<float>(i,j);
			if(depthValue > 0 )//&& depthValue < 20) 
			{
				newPoint.z = depthValue;
				newPoint.x = (j - depthIntrinsicMatrix(0,2)) * newPoint.z * depthFocalInvertedX;
				newPoint.y = (i - depthIntrinsicMatrix(1,2)) * newPoint.z * depthFocalInvertedY;
				outputPointCloud->push_back(newPoint);
			} 
			else {
				//std::cout << "Depthvalue: " << depthValue << std::endl;
				newPoint.z = std::numeric_limits<float>::quiet_NaN();
				newPoint.x = std::numeric_limits<float>::quiet_NaN();
				newPoint.y = std::numeric_limits<float>::quiet_NaN();
				outputPointCloud->push_back(newPoint);
			}
		}
	}
	outputPointCloud->height = h;
	outputPointCloud->width = w;
	outputPointCloud->is_dense = false;
	outputPointCloud->points.resize (w * h);
}

void createXYZRGBPointCloud_crop(int originx, int originy, int w, int h, cv::Mat& depthImage, cv::Mat& rgbImage, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud)
{
	

	pcl::PointXYZRGB newPoint;
	for (int i=originy;i<originy+h;i++)
	{
		for (int j=originx;j<originx+w;j++)
		{
			float depthValue = depthImage.at<float>(i,j);

			USHORT handDist = (USHORT)depthValue;
			long depthX = (long)j;
			long depthY = (long)i;
			Vector4 vector4 = NuiTransformDepthImageToSkeleton(depthX, depthY, handDist,NUI_IMAGE_RESOLUTION_640x480); 
			
			if(depthValue > 0)              // if depthValue is not NaN
			{
				newPoint.z = vector4.z;
				newPoint.x = vector4.x;
				newPoint.y = vector4.y;
				newPoint.r = rgbImage.at<cv::Vec3b>(i,j)[2];
				newPoint.g = rgbImage.at<cv::Vec3b>(i,j)[1];
				newPoint.b = rgbImage.at<cv::Vec3b>(i,j)[0];
				output_cloud->push_back(newPoint);
			}
			else
			{
				newPoint.z = std::numeric_limits<float>::quiet_NaN();
				newPoint.x = std::numeric_limits<float>::quiet_NaN();
				newPoint.y = std::numeric_limits<float>::quiet_NaN();
				newPoint.r = std::numeric_limits<unsigned char>::quiet_NaN();
				newPoint.g = std::numeric_limits<unsigned char>::quiet_NaN();
				newPoint.b = std::numeric_limits<unsigned char>::quiet_NaN();
				output_cloud->push_back(newPoint);
			}
		}
	}

	output_cloud->height = h;
	output_cloud->width = w;
	output_cloud->is_dense = false;
	output_cloud->points.resize (w * h);
}



void rotatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Eigen::Vector3f &normal)
{
	float c1 = sqrt(normal(1)*normal(1) + normal(2)*normal(2));
	float s1 = normal(0);

	float c2 = c1 ? normal(1) / c1 : 1.0;
	float s2 = c1 ? normal(2) / c1 : 0.0;

	Eigen::Matrix4f transformation;

	transformation <<	c1,			-s1*c2,			-s1*s2,		0,
						normal(0),	normal(1),		normal(2),	0,
						0,			-s2,			c2,			0,
						0,			0,				0,			1;

	pcl::transformPointCloud(*cloud, *cloud, transformation);
}

void reflectPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {

	Eigen::Matrix4f transformation;
	
	transformation <<	-1,		0,		0,		0,
						0,		-1,		0,		0,
						0,		0,		1,		0,
						0,		0,		0,		1;

	transformPointCloud(*cloud, *cloud, transformation);

}
void createXYZRGBPointCloud(cv::Mat& depthImage, cv::Mat& rgbImage, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud)
{
	pcl::PointXYZRGB newPoint;
	//double maxdepth=0;
	for (int i=0;i<depthImage.rows;i++)
	{
		for (int j=0;j<depthImage.cols;j++)
		{
			float depthValue = depthImage.at<float>(i,j);

			USHORT handDist = (USHORT)depthValue;
			long depthX = (long)j;
			long depthY = (long)i;
			Vector4 vector4 = NuiTransformDepthImageToSkeleton(depthX, depthY, handDist,NUI_IMAGE_RESOLUTION_640x480); 

			if(depthValue > 0)              // if depthValue is not NaN
			{
				// Find 3D position respect to rgb frame:
				newPoint.z = vector4.z;//depthValue;
				newPoint.x = vector4.x;//(j - rgb_intrinsic_matrix(0,2)) * newPoint.z * rgb_focal_inverted_x;
				newPoint.y = vector4.y;//(i - rgb_intrinsic_matrix(1,2)) * newPoint.z * rgb_focal_inverted_y;
				newPoint.r = rgbImage.at<cv::Vec3b>(i,j)[2];
				newPoint.g = rgbImage.at<cv::Vec3b>(i,j)[1];
				newPoint.b = rgbImage.at<cv::Vec3b>(i,j)[0];
				output_cloud->push_back(newPoint);
			}
			else
			{
				newPoint.z = std::numeric_limits<float>::quiet_NaN();
				newPoint.x = std::numeric_limits<float>::quiet_NaN();
				newPoint.y = std::numeric_limits<float>::quiet_NaN();
				newPoint.r = std::numeric_limits<unsigned char>::quiet_NaN();
				newPoint.g = std::numeric_limits<unsigned char>::quiet_NaN();
				newPoint.b = std::numeric_limits<unsigned char>::quiet_NaN();
				output_cloud->push_back(newPoint);
			}
			//}
		}
	}

	output_cloud->height = depthImage.rows;
	output_cloud->width = depthImage.cols;
	output_cloud->is_dense = false;
	output_cloud->points.resize (output_cloud->width * output_cloud->height);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void createXYZRGBPointCloud_keypoint(cv::Mat& depthImage, cv::Mat& rgbImage, std::vector<Vector4> &vec_keypoint, pcl::PointCloud<pcl::PointXYZ>::Ptr &vec_cloud_keypoint)
{
	pcl::PointXYZ newPoint;
	
	for (int p = 0; p < vec_keypoint.size(); p++)
	{
		////////////////////////////////////////////////
		for (int i = 0; i < depthImage.rows; i++)
		{
			for (int j = 0; j < depthImage.cols; j++)
			{
				int x = vec_keypoint[p].x;
				int y= vec_keypoint[p].y;

				if (i==y && j==x)
				{ 
					float depthValue = depthImage.at<float>(i, j);

					USHORT handDist = (USHORT)depthValue;
					long depthX = (long)j;
					long depthY = (long)i;
					Vector4 vector4 = NuiTransformDepthImageToSkeleton(depthX, depthY, handDist, NUI_IMAGE_RESOLUTION_640x480);

					if (depthValue > 0)              // if depthValue is not NaN
					{
						// Find 3D position respect to rgb frame:
						newPoint.z = vector4.z;//depthValue;
						newPoint.x = vector4.x;//
						newPoint.y = vector4.y;//
						vec_cloud_keypoint->push_back(newPoint);
					}
					else
					{
						int flag = 0;
						for (int k = i - 5; k < i + 5; k++)
						{
							for (int l = j - 5; l < j + 5; l++)
							{
								if (k > 0 && l > 0 && k < depthImage.rows && l < depthImage.cols &&  flag == 0)
								{
									float depthValue1 = depthImage.at<float>(k, l);
									if (depthValue1 > 0)
									{
										flag = 1;
										USHORT handDist1 = (USHORT)depthValue1;
										long depthX1 = (long)l;
										long depthY1 = (long)k;
										Vector4 vector41 = NuiTransformDepthImageToSkeleton(depthX1, depthY1, handDist1, NUI_IMAGE_RESOLUTION_640x480);
										newPoint.z = vector41.z;//;
										newPoint.x = vector41.x;//
										newPoint.y = vector41.y;//
										vec_cloud_keypoint->push_back(newPoint);
									}
									
								}
							}
						}
						
					}
			  }
			}
		}
		///////////////////////////////////////end each point//////////////////////////////
	}
	
}
///////////////////////////////////////////////////////////////////////////////////////////

void createXYZRGBPointCloud_dowsampling331(cv::Mat& depthImage, cv::Mat& rgbImage, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud)
{
	//double temp=INT_MIN;
	pcl::PointXYZRGB newPoint;
	for (int i=0;i<depthImage.rows;i++)
	{
		for (int j=0;j<depthImage.cols;j++)
		{

			if (((i%2)>0) && ((j%2)>0))
			{
				float AveDepth=0;
				int dem=0;
				for(int k=i-1;k<=i+1;k++)
				{
					if(k<480)
					{
					for(int l=j-1;l<=j+1;l++)
					{
						if (l<640)
						{
						float depthValuekl = depthImage.at<float>(k,l);
						if(depthValuekl>0)
							{
							AveDepth=AveDepth + depthValuekl;
							dem=dem+1;
							}
						}
					}
					}
				}
				AveDepth=AveDepth/dem;
				

				USHORT handDist = (USHORT)AveDepth;
				long depthX = (long)j;
				long depthY = (long)i;
				Vector4 vector4 = NuiTransformDepthImageToSkeleton(depthX, depthY, handDist,NUI_IMAGE_RESOLUTION_640x480); 
				
				if(AveDepth > 0 )              // if depthValue is not NaN
				{
					/*if (AveDepth>temp)
					{
						temp= AveDepth;*/
					// Find 3D position respect to rgb frame:
						newPoint.z = vector4.z;//depthValue;
						newPoint.x = vector4.x;//(j - rgb_intrinsic_matrix(0,2)) * newPoint.z * rgb_focal_inverted_x;
						newPoint.y = vector4.y;//(i - rgb_intrinsic_matrix(1,2)) * newPoint.z * rgb_focal_inverted_y;
						newPoint.r = rgbImage.at<cv::Vec3b>(i,j)[2];
						newPoint.g = rgbImage.at<cv::Vec3b>(i,j)[1];
						newPoint.b = rgbImage.at<cv::Vec3b>(i,j)[0];
						output_cloud->push_back(newPoint);
					//}
				}
				else
				{
					newPoint.z = std::numeric_limits<float>::quiet_NaN();
					newPoint.x = std::numeric_limits<float>::quiet_NaN();
					newPoint.y = std::numeric_limits<float>::quiet_NaN();
					newPoint.r = std::numeric_limits<unsigned char>::quiet_NaN();
					newPoint.g = std::numeric_limits<unsigned char>::quiet_NaN();
					newPoint.b = std::numeric_limits<unsigned char>::quiet_NaN();
					output_cloud->push_back(newPoint);
				}
			}	
		}
	}
	output_cloud->height =depthImage.rows/2;
	output_cloud->width = (depthImage.cols/2);
	output_cloud->is_dense = false;
	output_cloud->points.resize (output_cloud->width * output_cloud->height);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void createXYZRGBPointCloud_dowsampling331_crop(cv::Rect rect_object,cv::Mat& depthImage, cv::Mat& rgbImage, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud)
{
	//double temp=INT_MIN;
	int x=rect_object.x;
	int y=rect_object.y;
	int wid=rect_object.width;
	int hei=rect_object.height;
	

	pcl::PointXYZRGB newPoint;
	for (int i=y;i<y+hei;i++)
	{
		for (int j=x;j<x+wid;j++)
		{

			if (((i%2)>0) && ((j%2)>0))
			{
				float AveDepth=0;
				int dem=0;
				for(int k=i-1;k<=i+1;k++)
				{
					if(k<480)
					{
					for(int l=j-1;l<=j+1;l++)
					{
						if (l<640)
						{
						float depthValuekl = depthImage.at<float>(k,l);
						if(depthValuekl>0)
							{
							AveDepth=AveDepth + depthValuekl;
							dem=dem+1;
							}
						}
					}
					}
				}
				AveDepth=AveDepth/dem;
				

				USHORT handDist = (USHORT)AveDepth;
				long depthX = (long)j;
				long depthY = (long)i;
				Vector4 vector4 = NuiTransformDepthImageToSkeleton(depthX, depthY, handDist,NUI_IMAGE_RESOLUTION_640x480); 
				
				if(AveDepth > 0 )              // if depthValue is not NaN
				{
					/*if (AveDepth>temp)
					{
						temp= AveDepth;*/
					// Find 3D position respect to rgb frame:
						newPoint.z = vector4.z;//depthValue;
						newPoint.x = vector4.x;//(j - rgb_intrinsic_matrix(0,2)) * newPoint.z * rgb_focal_inverted_x;
						newPoint.y = vector4.y;//(i - rgb_intrinsic_matrix(1,2)) * newPoint.z * rgb_focal_inverted_y;
						newPoint.r = rgbImage.at<cv::Vec3b>(i,j)[2];
						newPoint.g = rgbImage.at<cv::Vec3b>(i,j)[1];
						newPoint.b = rgbImage.at<cv::Vec3b>(i,j)[0];
						output_cloud->push_back(newPoint);
					//}
				}
				else
				{
					newPoint.z = std::numeric_limits<float>::quiet_NaN();
					newPoint.x = std::numeric_limits<float>::quiet_NaN();
					newPoint.y = std::numeric_limits<float>::quiet_NaN();
					newPoint.r = std::numeric_limits<unsigned char>::quiet_NaN();
					newPoint.g = std::numeric_limits<unsigned char>::quiet_NaN();
					newPoint.b = std::numeric_limits<unsigned char>::quiet_NaN();
					output_cloud->push_back(newPoint);
				}
			}	
		}
	}
	output_cloud->height =(y+hei)/2;
	output_cloud->width = ((x+wid)/2);
	output_cloud->is_dense = false;
	output_cloud->points.resize (output_cloud->width * output_cloud->height);
}
///////////////////////////////////////////////////////////////////////////
void createXYZRGBPointCloud_dowsampling332(cv::Mat& depthImage, cv::Mat& rgbImage, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud)
{
	//double temp=INT_MIN;
	pcl::PointXYZRGB newPoint;
	for (int i=0;i<depthImage.rows;i++)
	{
		for (int j=0;j<depthImage.cols;j++)
		{

			if (((i%2)>0) && ((j%2)>0))
			{
				float AveDepth=0;
				int dem=0;
				for(int k=i-1;k<=i+1;k++)
				{
					if(k<480)
					{
					for(int l=j-1;l<=j+1;l++)
					{
						if (l<640)
						{
						float depthValuekl = depthImage.at<float>(k,l);
						if(depthValuekl>0)
							{
							AveDepth=AveDepth + depthValuekl;
							dem=dem+1;
							}
						}
					}
					}
				}
				AveDepth=AveDepth/dem;
				

				USHORT handDist = (USHORT)AveDepth;
				long depthX = (long)j;
				long depthY = (long)i;
				Vector4 vector4 = NuiTransformDepthImageToSkeleton(depthX, depthY, handDist,NUI_IMAGE_RESOLUTION_640x480); 
				
				if(AveDepth > 0 && AveDepth <1600)              // if depthValue is not NaN
				{
					/*if (AveDepth>temp)
					{
						temp= AveDepth;*/
					// Find 3D position respect to rgb frame:
						newPoint.z = vector4.z;//depthValue;
						newPoint.x = vector4.x;//(j - rgb_intrinsic_matrix(0,2)) * newPoint.z * rgb_focal_inverted_x;
						newPoint.y = vector4.y;//(i - rgb_intrinsic_matrix(1,2)) * newPoint.z * rgb_focal_inverted_y;
						newPoint.r = rgbImage.at<cv::Vec3b>(i,j)[2];
						newPoint.g = rgbImage.at<cv::Vec3b>(i,j)[1];
						newPoint.b = rgbImage.at<cv::Vec3b>(i,j)[0];
						output_cloud->push_back(newPoint);
					//}
				}
				else
				{
					newPoint.z = std::numeric_limits<float>::quiet_NaN();
					newPoint.x = std::numeric_limits<float>::quiet_NaN();
					newPoint.y = std::numeric_limits<float>::quiet_NaN();
					newPoint.r = std::numeric_limits<unsigned char>::quiet_NaN();
					newPoint.g = std::numeric_limits<unsigned char>::quiet_NaN();
					newPoint.b = std::numeric_limits<unsigned char>::quiet_NaN();
					output_cloud->push_back(newPoint);
				}
			}	
		}
	}
	output_cloud->height =depthImage.rows/2 ;
	output_cloud->width = (depthImage.cols/2);
	output_cloud->is_dense = false;
	output_cloud->points.resize (output_cloud->width * output_cloud->height);
}
void createXYZPointCloud(cv::Mat& depthImage, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPointCloud)
{

	Eigen::Matrix3f depthIntrinsicMatrix;

	// ===================================
	//		Nathan Silberman parameters
	// ===================================
	depthIntrinsicMatrix(0,0) = 582.62448167737955f; //fx
	depthIntrinsicMatrix(1,1) = 582.69103270988637f; //fy
	depthIntrinsicMatrix(0,2) = 313.04475870804731f; //cx
	depthIntrinsicMatrix(1,2) = 238.44389626620386f; //cy

	float depthFocalInvertedX = 1/depthIntrinsicMatrix(0,0);	// 1/fx
	float depthFocalInvertedY = 1/depthIntrinsicMatrix(1,1);	// 1/fy

	pcl::PointXYZ newPoint;

	for (int i=0;i<depthImage.rows;i++)
	{
		for (int j=0;j<depthImage.cols;j++)
		
		{
			float depthValue = depthImage.at<float>(i,j);
			if(depthValue > 0 )//&& depthValue < 20) 
			{
				newPoint.z = depthValue;
				newPoint.x = (j - depthIntrinsicMatrix(0,2)) * newPoint.z * depthFocalInvertedX;
				newPoint.y = (i - depthIntrinsicMatrix(1,2)) * newPoint.z * depthFocalInvertedY;
				outputPointCloud->push_back(newPoint);
			} 
			else {
				//std::cout << "Depthvalue: " << depthValue << std::endl;
				newPoint.z = std::numeric_limits<float>::quiet_NaN();
				newPoint.x = std::numeric_limits<float>::quiet_NaN();
				newPoint.y = std::numeric_limits<float>::quiet_NaN();
				outputPointCloud->push_back(newPoint);
			}
		}
	}
	outputPointCloud->height = depthImage.rows;
	outputPointCloud->width = depthImage.cols;
	outputPointCloud->is_dense = false;
	outputPointCloud->points.resize (outputPointCloud->width * outputPointCloud->height);
}
