#include"include/visualizer.h"

//#include <iostream>
//#include<pcl/visualization/cloud_viewer.h>

typedef struct RandomColor
	{
	  int R,G,B;
	} RandomColor;

RandomColor FunctionRandomColor()
{
	RandomColor colorout;
	colorout.R = rand() % 255; 
	colorout.G = rand() % 255; 
	colorout.B = rand() % 255; 
	return colorout;
}
void visualization_scene::visualiall(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene,std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters, std::vector<int> task, int frame)
{
	 viewer->removeAllPointClouds();
	 viewer->removeAllShapes();
	 viewer->removeCoordinateSystem();
	 viewer->removeCorrespondences();
	 char filescene[100];
	 sprintf(filescene,"scene(%d)",frame);
	 int v1(0);
	 std::stringstream sstm;
	 sstm << filescene;
	 viewer->setBackgroundColor (0, 0, 0, v1);
	 viewer->addText(sstm.str(), 10, 10, "v1 text", v1);

	 /*pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_scene);
   viewer->addPointCloud<pcl::PointXYZRGB> (cloud_scene, rgb, filescene, v1);*/


	 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_scene, 255, 255,255);
	 viewer->addPointCloud<pcl::PointXYZ> (cloud_scene, single_color, filescene);
	 viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, filescene);
	char file[100];
	 for(int i=0;i<clusters.size();i++)
	{
		sprintf(file,"object_%d_%d",frame,i);
		if (task[i]==1)
		{
			 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(clusters[i], 255, 0, 0);
			 viewer->addPointCloud<pcl::PointXYZ> (clusters[i], single_color, file);
		}
		else if(task[i]==2)
		{
			 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(clusters[i], 0, 255, 0);
			 viewer->addPointCloud<pcl::PointXYZ> (clusters[i], single_color, file);
		}
		else
		{
			 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(clusters[i], 255, 255, 255);
			 viewer->addPointCloud<pcl::PointXYZ> (clusters[i], single_color, file);
		}
		 viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, file);
	}

  viewer->addCoordinateSystem (1.0);
   /*while(!viewer->wasStopped())
  {*/
	  //viewer->sa
		viewer->spinOnce (1000);
		//boost::this_thread::sleep (boost::posix_time::microseconds (100000));
   //}
}
void visualization_scene::visualiall_multiplane(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene,std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters,int frame)
{
	// viewer->removeAllPointClouds();
	// viewer->removeAllShapes();
	// viewer->removeCoordinateSystem();
	// viewer->removeCorrespondences();
	// char filescene[100];
	// sprintf(filescene,"scene(%d)",frame);
	// int v1(0);
	// stringstream sstm;
	// sstm << filescene;
	// viewer->setBackgroundColor (0, 0, 0, v1);
	// viewer->addText(sstm.str(), 10, 10, "v1 text", v1);

	// /*pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_scene);
 //  viewer->addPointCloud<pcl::PointXYZRGB> (cloud_scene, rgb, filescene, v1);*/


	///* pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_scene, 0, 255,0);
	// viewer->addPointCloud<pcl::PointXYZ> (cloud_scene, single_color, filescene);
	// viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, filescene);*/
	//char file[100], filenormal[100];
	// for(int i=0;i<clusters.size();i++)
	//{
	//		 sprintf(file,"object_%d",i);
	//		 RandomColor color;
	//		 color=FunctionRandomColor();
	//		 
	//		 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(clusters[i], color.R, color.G, color.B);
	//		 viewer->addPointCloud<pcl::PointXYZ> (clusters[i], single_color, file);
	//	     viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, file);
	//		 ////viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (clusters[i], clusters_normal[i], 10, 1, filenormal, v1);

	//}
	//
	//viewer->addCoordinateSystem (10);
 // /* while(!viewer->wasStopped())
 // {*/
	//  //viewer->sa
	//if (frame==1)
	//{
	//	viewer->spinOnce (10000);//10000);
	//}
	//else
	//{
	//	viewer->spinOnce (0);
	//}
		
		//boost::this_thread::sleep (boost::posix_time::microseconds (100000));
   //}
}
void visualization_scene::visualiall_multiplanecolor(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data_r,
	pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals,std::vector<int> sampleselect,
	pcl::ModelCoefficients &cylinder_coefficitent,int type_model,int numdata,int frame,int iter_run,
	char filename[1000],char filenamer[1000], char filenamecylinder[1000])
{
	  viewer->removeAllPointClouds();
	 viewer->removeAllShapes();
	 viewer->removeCoordinateSystem();
	 viewer->removeCorrespondences();


	 char fileimagesave[1000];

	 char filesall[1000], filesinli[1000],filecylinder[1000];
	 sprintf(filesall,"scene_data%d_%d_%d_%d",numdata,frame,iter_run,type_model);
	  sprintf(filesinli,"outlier%d_%d_%d_%d",numdata,frame,iter_run,type_model);
	  sprintf(filecylinder,"cylinder_data%d_%d_%d_%d",numdata,frame,iter_run,type_model);
	  
	
	/*stringstream sstm;
	 sstm << filecylinder;
	 viewer->setBackgroundColor (0, 0, 0, v1);
	 viewer->addText(sstm.str(), 10, 10, "v1 text", v1);*/

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_data, 255, 0, 0);
		viewer->addPointCloud<pcl::PointXYZ> (cloud_data, single_color, filesall);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, filesall);

		/*viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud_data, cloud_normals, 1, 1, filenormal, v1);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,filenormal,v1);*/

		/*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cloud_data_r, 0, 0, 255);
		viewer->addPointCloud<pcl::PointXYZ> (cloud_data_r, single_color1, filesinli);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, filesinli);*/
		//draw line 




			viewer->addCylinder(cylinder_coefficitent,filecylinder,0);
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.0,1.0,0.0,filecylinder,0);
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,50,filecylinder,0);

			/*viewer->addSphere(cylinder_coefficitent,filecylinder,0);
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.0,1.0,0.0,filecylinder,0);
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,50,filecylinder,0);*/

            /*viewer->addCone(cylinder_coefficitent,filecylinder,0);
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.0,1.0,0.0,filecylinder,0);
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,50,filecylinder,0);*/
  //mlesac
  if (type_model==1)
  {
	  sprintf(fileimagesave,"D:/3D_object_DATASET/rgbd-dataset_pcd/food_can/result_food_can%d/result_MLESAC/image/source/frame_source_%d_iterrun%d_object_%d.png",numdata,numdata,iter_run, frame);
  }
  //impro
  else if(type_model==2)
  {
	   sprintf(fileimagesave,"D:/3D_object_DATASET/rgbd-dataset_pcd/food_can/result_food_can%d/result_MLESAC/image/impro/frame_impro_%d_iterrun%d_object_%d.png",numdata,numdata,iter_run, frame);
  }
  //napsac
  else if(type_model==3)
  {
	  sprintf(fileimagesave,"D:/3D_object_DATASET/rgbd-dataset_pcd/food_can/result_food_can%d/result_NAPSAC/image/frame_napsac_%d_iterrun%d_object_%d.png",numdata,numdata,iter_run, frame);
  }
  //ransac
   if(type_model==4)
  {
	  sprintf(fileimagesave,"D:/3D_object_DATASET/rgbd-dataset_pcd/food_can/result_food_can%d/result_RANSAC/image/frame_ransac_%d_iterrun%d_object_%d.png",numdata,numdata,iter_run, frame);
  }
   //prosac
   else if(type_model==5)
  {
	  sprintf(fileimagesave,"D:/3D_object_DATASET/rgbd-dataset_pcd/food_can/result_food_can%d/result_PROSAC/image/frame_prosac_%d_iterrun%d_object_%d.png",numdata,numdata,iter_run, frame);
  }
   //msac
   else if(type_model==6)
  {
	  sprintf(fileimagesave,"D:/3D_object_DATASET/rgbd-dataset_pcd/food_can/result_food_can%d/result_MSAC/image/frame_msac_%d_iterrun%d_object_%d.png",numdata,numdata,iter_run, frame);
  }
   //losac
  else if(type_model==7)
  {
	  sprintf(fileimagesave,"D:/3D_object_DATASET/rgbd-dataset_pcd/food_can/result_food_can%d/result_LOSAC/image/frame_losac_%d_iterrun%d_object_%d.png",numdata,numdata,iter_run, frame);
  }

  viewer->addCoordinateSystem (0.1);
  viewer->setBackgroundColor(255.0,255.0,255.0);
   while(!viewer->wasStopped())
  {
		viewer->spinOnce(10);
		
  }
  viewer->saveScreenshot(fileimagesave); 

     viewer->removeAllPointClouds();
	 viewer->removeAllShapes();
	 viewer->removeCoordinateSystem();
	 viewer->removeCorrespondences();
}



void visualization_scene::visualiall_multiplane_Normal(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters)
{
	 viewer->removeAllPointClouds();
	 viewer->removeAllShapes();
	 viewer->removeCoordinateSystem();
	 viewer->removeCorrespondences();
	 char filescene[100];
	 //sprintf(filescene,"scene(%d)",frame);
	 int v1(0);
	 stringstream sstm;
	 sstm << filescene;
	 viewer->setBackgroundColor (0, 0, 0, v1);
	 viewer->addText(sstm.str(), 10, 10, "v1 text", v1);
	char file[100];


	 for(int i=0;i<clusters.size();i++)
	{
			 /*sprintf(file,"object_%d",i);
						  RandomColor color;
			 color=FunctionRandomColor();
			 
			 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(clusters[i], color.R, color.G, color.B);
			 viewer->addPointCloud<pcl::PointXYZ> (clusters[i], single_color, file);
		     viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, file);*/
			 

	}
	
	viewer->addCoordinateSystem (1.0);
   while(!viewer->wasStopped())
  {
	  //viewer->sa
		viewer->spinOnce (1000);
		//boost::this_thread::sleep (boost::posix_time::microseconds (100000));
   }
}
void visualization_scene::visuali(char filewrite[200],boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,int frame,int startframe)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->removeAllPointClouds();
	 viewer->removeAllShapes();
	 viewer->removeCoordinateSystem();
	 viewer->removeCorrespondences();
	char filescene[100],file[100];
	 sprintf(filescene,"scene(%d)",frame);
	 sprintf(file,"tablescene(%d)",frame);
	 int v1(0);
	 stringstream sstm;
	 sstm << "Frame:  "<<frame;
	 viewer->setBackgroundColor (0, 0, 0, v1);
	 viewer->addText(sstm.str(), 10, 10, "v1 text", v1);
  //viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  //viewer->setBackgroundColor (0.3, 0.3, 0.3, v1);
 // viewer->setBackgroundColor (0, 0, 0);
   pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	 viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, filescene);
	 viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, filescene);

  viewer->addCoordinateSystem (1.0);
  //viewer->initCameraParameters ();
  /*while(!viewer->wasStopped())
  {*/
	  if (frame==startframe)
	  {
		   viewer->spinOnce (10000);
	  }
	  else
	  {
		   viewer->spinOnce (0);
	  }
	 
	   viewer->saveScreenshot(filewrite); 
 //}

  //return (viewer);
}


void visualization_scene::visuali_two(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1,int frame)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->removeAllPointClouds();
	 viewer->removeAllShapes();
	 viewer->removeCoordinateSystem();
	 viewer->removeCorrespondences();
	char filescene[100],file[100];
	 sprintf(filescene,"scene(%d)",frame);
	 sprintf(file,"tablescene(%d)",frame);
	 int v1(0);
	 stringstream sstm;
	 sstm << "Frame:  "<<frame;
	 viewer->setBackgroundColor (0, 0, 0, v1);
	 viewer->addText(sstm.str(), 10, 10, "v1 text", v1);

   /*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255,255, 255);
 viewer->addPointCloud<pcl::PointXYZ> (cloud ,single_color, filescene);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, filescene);*/

	  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	 viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, filescene);
	 viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, filescene);


  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cloud1, 255,0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud1 ,single_color1, file);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,10, file);

		/*char name[1000],name1[1000];
		Eigen::Vector3f centroid (cloud1->points[(int) (cloud1->points.size()/2)].x,cloud1->points[(int) (cloud1->points.size()/2)].y,cloud1->points[(int) (cloud1->points.size()/2)].z);
		Eigen::Vector3f model (-0.19954203,-0.95408791,-0.22338115);

		Eigen::Vector3f model1 (0,-1,0);

			pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
			


			pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (1.0f * model[0]),
											   centroid[1] + (1.0f * model[1]),
											   centroid[2] + (1.0f * model[2]));

			pcl::PointXYZ pt3 = pcl::PointXYZ (centroid[0] + (1.0f * model1[0]),
											   centroid[1] + (1.0f * model1[1]),
											   centroid[2] + (1.0f * model1[2]));


			sprintf (name, "normal_%d", unsigned (11));
			sprintf (name1, "normal1_%d", unsigned (12));
			viewer->addArrow (pt2, pt1, 1.0, 0, 0, false, name);
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);

			viewer->addArrow (pt3, pt1, 0, 0, 1.0, false, name1);
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name1);*/

  //values = [4](-0.19954203,-0.95408791,-0.22338115,4.3336725)



  viewer->addCoordinateSystem (1.0);

  while(!viewer->wasStopped())
  {
	   viewer->spinOnce (0);
	  //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 }

}
void  visualization_scene::visuali_two_color(int startfram,int mohinh, int dataso, boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudobject,std::vector<pcl::ModelCoefficients::Ptr> vect_coefficitentmodel, int frame)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->removeAllPointClouds();
	 viewer->removeAllShapes();
	 viewer->removeCoordinateSystem();
	 viewer->removeCorrespondences();
	viewer->removeCoordinateSystem();
	viewer->removeText3D();
	
	int v1(0);
	/* stringstream sstm;
	 sstm << "Frame:  "<<frame;*/
	 viewer->setBackgroundColor (0, 0, 0, v1);
	 char filescene[100], file1[100], file2[100],fileobject1[100], fileobject2[100];
	 sprintf(filescene,"scene(%d)",frame);
	 sprintf(file1,"file1(%d)",frame);
	 sprintf(file2,"file2(%d)",frame);
	 //sprintf(fileobject1, "object1(%d)", frame);
	 //sprintf(fileobject2, "object2(%d)", frame);
	 //int v1(0);
	 stringstream sstm;
	 sstm << "Frame:  "<<frame;
	 //draw line 

	 pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	 viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, filescene);
	 viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, filescene);

	 pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(cloudobject);
	 viewer->addPointCloud<pcl::PointXYZRGB>(cloudobject, rgb1, file1);
	 viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, file1);


	if (mohinh == 2)//cylinder
	{
	
		pcl::PointCloud<pcl::PointXYZ>::Ptr colorvalue(new	pcl::PointCloud<pcl::PointXYZ>);
		colorvalue->points.resize(5);
		colorvalue->points[0].x=255;
		colorvalue->points[0].y=0;
		colorvalue->points[0].z=0;

		colorvalue->points[1].x=0;
		colorvalue->points[1].y=255;
		colorvalue->points[1].z=0;

		colorvalue->points[2].x=0;
		colorvalue->points[2].y=0;
		colorvalue->points[2].z=255;

		colorvalue->points[3].x=255;
		colorvalue->points[3].y=255;
		colorvalue->points[3].z=0;


		colorvalue->points[4].x=255;
		colorvalue->points[4].y=0;
		colorvalue->points[4].z=255;

		for(int vung=0;vung< vect_coefficitentmodel.size();vung++)
		{
			sprintf(fileobject1, "object1_%d_%d", frame,vung);
			viewer->addCylinder(*vect_coefficitentmodel[vung], fileobject1, 0);
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colorvalue->points[vung].x, colorvalue->points[vung].y, colorvalue->points[vung].z, fileobject1, 0);
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, fileobject1, 0);
		}
	  
	 }
	  //viewer->addCoordinateSystem (1.0);
	 viewer->setBackgroundColor (1.0, 1.0, 1.0, v1);
	  char fileimage[500];
	  if (mohinh == 1)
	  {
		  sprintf(fileimage, "G:/3D_object_DATASET/3dobject_29_9/Data_captured%d/object_ceoplane_pointcloud/find_cylindrical_Object_stage2/ourmethod/pointcloud/Image_cylinder_%d.png", dataso,frame);
	  }
	  else if (mohinh == 2)
	  {
		  sprintf(fileimage, "G:/rgbd_scenes_v2_imgs/scene_%02d/pointclouddata/result_findcylinder_CM/pointcloud/Image_cylinder_%d.png", dataso,frame);
	  }
	  
	  /*while(!viewer->wasStopped())
	  {*/
	  if(frame==startfram)
	  {
		  viewer->spinOnce (5000);
	  }
	  else
	  {
		  viewer->spinOnce (0);
	  }
	 //}
	  viewer->saveScreenshot(fileimage);
		  //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	  

}

void visualization_scene:: visuali_two_color_vectorcloud(char fileimagewrite[500], boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, 
		pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_scene, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> vector_cloudobject,
		std::vector<int> &vector_typeoject, std::vector<pcl::ModelCoefficients::Ptr> &Vector_ceoficientobject, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr tableplane, pcl::ModelCoefficients::Ptr &ceoplane,
		int frame,int startframe)
{
	  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
     viewer->removeAllPointClouds();
	 viewer->removeAllShapes();
	 viewer->removeCoordinateSystem();
	 viewer->removeCorrespondences();
	viewer->removeCoordinateSystem();
	viewer->removeText3D();
	
	int v1(0);
	/* stringstream sstm;
	 sstm << "Frame:  "<<frame;*/
	 viewer->setBackgroundColor (0, 0, 0, v1);

	 char filescene[100],filetableplane[100], file1[100], file2[100],file3[100],file4[100],file5[100], 
		 filecylin1[100], filecylin2[100],filecylin3[100],filecylin4[100],filecylin5[100],
		 fileobject1[100], fileobject2[100],namenormalplane[200],filenormal1[200], filenormal2[200];

	 sprintf(filescene,"scene(%d)",frame);
	 sprintf(filetableplane,"tableplane(%d)",frame);

	 sprintf(file1,"file1(%d)",frame);
	 sprintf(file2,"file2(%d)",frame);
	 sprintf(file3,"file3(%d)",frame);
	 sprintf(file4,"file4(%d)",frame);
	 sprintf(file5,"file5(%d)",frame);

	 sprintf(filecylin1,"filecylin1(%d)",frame);
	 sprintf(filecylin2,"filecylin2(%d)",frame);
	 sprintf(filecylin3,"filecylin3(%d)",frame);
	 sprintf(filecylin4,"filecylin4(%d)",frame);
	 sprintf(filecylin5,"filecylin5(%d)",frame);

	 
	 stringstream sstm;
	 sstm << "Frame:  "<<frame;
	 //draw line 

	/* pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_scene);
	 viewer->addPointCloud<pcl::PointXYZRGB>(cloud_scene, rgb, filescene);
	 viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, filescene);*/

	  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_tablepane(tableplane, 255,0, 255);
	  viewer->addPointCloud<pcl::PointXYZ> (tableplane ,single_tablepane, filetableplane);
	  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, filetableplane);


	  Eigen::Vector3f centroid (tableplane->points[0].x,tableplane->points[0].y,tableplane->points[0].z);
	  Eigen::Vector3f model (-ceoplane->values[0],-ceoplane->values[1],-ceoplane->values[2]);

	 pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
	 pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (0.5f * model[0]),
											   centroid[1] + (0.5f * model[1]),
											   centroid[2] + (0.5f * model[2]));
	 sprintf (namenormalplane, "normal_%d", unsigned (1));
	 viewer->addArrow (pt2, pt1, 1.0, 0, 0, false, namenormalplane);
	 viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, namenormalplane);

	  char filetex1[100],filetex2[100],filetex3[100],filetex4[100],filetex5[100];
	  char filelocation1[100],filelocation2[100],filelocation3[100],filelocation4[100],filelocation5[100];
	  sprintf(filelocation1,"filelocation1(%d)",frame);
	 sprintf(filelocation2,"filelocation2(%d)",frame);
	 sprintf(filelocation3,"filelocation3(%d)",frame);
	 sprintf(filelocation4,"filelocation4(%d)",frame);
	 sprintf(filelocation5,"filelocation5(%d)",frame);


	 sprintf(filenormal1,"filenormal1(%d)",frame);
	 sprintf(filenormal2,"filenormal2(%d)",frame);


	 	 std::vector<color_value> array_color;
	 for (int i=0;i<vector_cloudobject.size();i++)
	 {
		 color_value color;
		 if (vector_typeoject[i]==7)
		 {
			 color.r=255;
			 color.g=255;
			 color.b=0;
		 }
		 else if (vector_typeoject[i]==8)
		 {
			 color.r=255;
			 color.g=255;
			 color.b=0;
		 }
		 else if (vector_typeoject[i]==8)
		 {
			 color.r=255;
			 color.g=255;
			 color.b=0;
		 }
		 else if (vector_typeoject[i]==9)
		 {
			 color.r=255;
			 color.g=255;
			 color.b=0;
		 }
		 else if (vector_typeoject[i]==10)
		 {
			 color.r=255;
			 color.g=0;
			 color.b=0;
		 }
		 else if (vector_typeoject[i]==11)
		 {
			 color.r=0;
			 color.g=255;
			 color.b=0;
		 }
		 else if (vector_typeoject[i]==12)
		 {
			 color.r=255;
			 color.g=255;
			 color.b=0;
		 }
		 else if (vector_typeoject[i]==13)
		 {
			 color.r=0;
			 color.g=0;
			 color.b=255;
		 }
		 else if (vector_typeoject[i]==14)
		 {
			 color.r=255;
			 color.g=255;
			 color.b=0;
		 }
		 array_color.push_back(color);
	 }


	 if(Vector_ceoficientobject.size()==1)
	 {
		 pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(vector_cloudobject[0]);
		 viewer->addPointCloud<pcl::PointXYZRGB>(vector_cloudobject[0], rgb, file1);
		 viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, file1);

		/* pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
		 estimating_normal es;
			 es.estimation(vector_cloudobject[0],cloud_normal);

		  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (vector_cloudobject[0], cloud_normal, 1, 0.2, filenormal1, v1);
		  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1.0,1.0,0.0,filenormal1,v1);*/


		  if(vector_typeoject[0]==2||vector_typeoject[0]==3||vector_typeoject[0]==4||vector_typeoject[0]==5||vector_typeoject[0]==6
					 ||vector_typeoject[0]==12||vector_typeoject[0]==13||vector_typeoject[0]==14||vector_typeoject[0]==16
					 ||vector_typeoject[0]==17||vector_typeoject[0]==18||vector_typeoject[0]==19||vector_typeoject[0]==20)
		  {
			viewer->addCylinder(*Vector_ceoficientobject[0],filecylin1,0);
			viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,filecylin1);
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,array_color[0].r,array_color[0].g,array_color[0].b,filecylin1,0);
			
		     if(vector_typeoject[0]==12)
			  {
				  sprintf(filetex1,"Paper Cup (%5.2f, %5.2f, %5.2f), Radius: %5.2f(m)\n", Vector_ceoficientobject[0]->values[0], Vector_ceoficientobject[0]->values[1],Vector_ceoficientobject[0]->values[2],Vector_ceoficientobject[0]->values[6]);
			  }
			  else if(vector_typeoject[0]==13)
			  {
				  sprintf(filetex1,"Medicine Vial (%5.2f, %5.2f, %5.2f), Radius: %5.2f(m)\n", Vector_ceoficientobject[0]->values[0], Vector_ceoficientobject[0]->values[1],Vector_ceoficientobject[0]->values[2],Vector_ceoficientobject[0]->values[6]);
			  }
			  else if(vector_typeoject[0]==14)
			  {
				  sprintf(filetex1,"Milk Bottle (%5.2f,%5.2f, %5.2f), Radius: %5.2f(m)\n", Vector_ceoficientobject[0]->values[0], Vector_ceoficientobject[0]->values[1],Vector_ceoficientobject[0]->values[2],Vector_ceoficientobject[0]->values[6]);
			  }
			  
		  
		  }
		  viewer->addText(filetex1,400,100,30,0.0,0.0,0.0,filelocation1);
	 }
	 else if(Vector_ceoficientobject.size()==2)
	 {

		 pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(vector_cloudobject[0]);
		 viewer->addPointCloud<pcl::PointXYZRGB>(vector_cloudobject[0], rgb, file1);
		 viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, file1);

		 if(vector_typeoject[0]==2||vector_typeoject[0]==3||vector_typeoject[0]==4||vector_typeoject[0]==5||vector_typeoject[0]==6
					 ||vector_typeoject[0]==12||vector_typeoject[0]==13||vector_typeoject[0]==14||vector_typeoject[0]==16
					 ||vector_typeoject[0]==17||vector_typeoject[0]==18||vector_typeoject[0]==19||vector_typeoject[0]==20)
		  {
			viewer->addCylinder(*Vector_ceoficientobject[0],filecylin1,0);
			viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,filecylin1);
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,array_color[0].r,array_color[0].g,array_color[0].b,filecylin1,0);
			if(vector_typeoject[0]==12)
			  {
				  sprintf(filetex1,"Paper Cup (%5.2f, %5.2f, %5.2f), Radius: %5.2f(m)\n", Vector_ceoficientobject[0]->values[0], Vector_ceoficientobject[0]->values[1],Vector_ceoficientobject[0]->values[2],Vector_ceoficientobject[0]->values[6]);
			  }
			  else if(vector_typeoject[0]==13)
			  {
				  sprintf(filetex1,"Medicine Vial (%5.2f, %5.2f, %5.2f), Radius: %5.2f(m)\n", Vector_ceoficientobject[0]->values[0], Vector_ceoficientobject[0]->values[1],Vector_ceoficientobject[0]->values[2],Vector_ceoficientobject[0]->values[6]);
			  }
			  else if(vector_typeoject[0]==14)
			  {
				  sprintf(filetex1,"Milk Bottle (%5.2f,%5.2f, %5.2f), Radius: %5.2f(m)\n", Vector_ceoficientobject[0]->values[0], Vector_ceoficientobject[0]->values[1],Vector_ceoficientobject[0]->values[2],Vector_ceoficientobject[0]->values[6]);
			  }
		  }
		 viewer->addText(filetex1,400,100,30,0.0,0.0,0.0,filelocation1);

		 pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(vector_cloudobject[1]);
		 viewer->addPointCloud<pcl::PointXYZRGB>(vector_cloudobject[1], rgb1, file2);
		 viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, file2);

		 if(vector_typeoject[1]==2||vector_typeoject[1]==3||vector_typeoject[1]==4||vector_typeoject[1]==5||vector_typeoject[1]==6
					 ||vector_typeoject[1]==12||vector_typeoject[1]==13||vector_typeoject[1]==14||vector_typeoject[1]==16
					 ||vector_typeoject[1]==17||vector_typeoject[1]==18||vector_typeoject[1]==19||vector_typeoject[1]==20)
		 {
			 viewer->addCylinder(*Vector_ceoficientobject[1],filecylin2,0);
			 viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,filecylin2);
		     viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,array_color[1].r,array_color[1].g,array_color[1].b,filecylin2,0);
			 if(vector_typeoject[1]==12)
			  {
				  sprintf(filetex2,"Paper Cup (%5.2f, %5.2f, %5.2f), Radius: %5.2f(m)\n", Vector_ceoficientobject[1]->values[0], Vector_ceoficientobject[1]->values[1],Vector_ceoficientobject[1]->values[2],Vector_ceoficientobject[1]->values[6]);
			  }
			  else if(vector_typeoject[1]==13)
			  {
				  sprintf(filetex2,"Medicine Vial (%5.2f, %5.2f, %5.2f), Radius: %5.2f(m)\n", Vector_ceoficientobject[1]->values[0], Vector_ceoficientobject[1]->values[1],Vector_ceoficientobject[1]->values[2],Vector_ceoficientobject[1]->values[6]);
			  }
			  else if(vector_typeoject[1]==14)
			  {
				  sprintf(filetex2,"Milk Bottle (%5.2f,%5.2f, %5.2f), Radius: %5.2f(m)\n", Vector_ceoficientobject[1]->values[0], Vector_ceoficientobject[1]->values[1],Vector_ceoficientobject[1]->values[2],Vector_ceoficientobject[1]->values[6]);
			  }
		 }
		 
		 viewer->addText(filetex2,400,145,30,0.0,0.0,0.0,filelocation2);
	 }
	else if(Vector_ceoficientobject.size()==3)
	 {

		 pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(vector_cloudobject[0]);
		 viewer->addPointCloud<pcl::PointXYZRGB>(vector_cloudobject[0], rgb, file1);
		 viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, file1);

		 if(vector_typeoject[0]==2||vector_typeoject[0]==3||vector_typeoject[0]==4||vector_typeoject[0]==5||vector_typeoject[0]==6
					 ||vector_typeoject[0]==12||vector_typeoject[0]==13||vector_typeoject[0]==14||vector_typeoject[0]==16
					 ||vector_typeoject[0]==17||vector_typeoject[0]==18||vector_typeoject[0]==19||vector_typeoject[0]==20)
		  {
			viewer->addCylinder(*Vector_ceoficientobject[0],filecylin1,0);
			viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,filecylin1);
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,array_color[0].r,array_color[0].g,array_color[0].b,filecylin1,0);
			if(vector_typeoject[0]==12)
			  {
				  sprintf(filetex1,"Paper Cup (%5.2f, %5.2f, %5.2f), Radius: %5.2f(m)\n", Vector_ceoficientobject[0]->values[0], Vector_ceoficientobject[0]->values[1],Vector_ceoficientobject[0]->values[2],Vector_ceoficientobject[0]->values[6]);
			  }
			  else if(vector_typeoject[0]==13)
			  {
				  sprintf(filetex1,"Medicine Vial (%5.2f, %5.2f, %5.2f), Radius: %5.2f(m)\n", Vector_ceoficientobject[0]->values[0], Vector_ceoficientobject[0]->values[1],Vector_ceoficientobject[0]->values[2],Vector_ceoficientobject[0]->values[6]);
			  }
			  else if(vector_typeoject[0]==14)
			  {
				  sprintf(filetex1,"Milk Bottle (%5.2f,%5.2f, %5.2f), Radius: %5.2f(m)\n", Vector_ceoficientobject[0]->values[0], Vector_ceoficientobject[0]->values[1],Vector_ceoficientobject[0]->values[2],Vector_ceoficientobject[0]->values[6]);
			  }
		  }
		 viewer->addText(filetex1,400,100,30,0.0,0.0,0.0,filelocation1);

		 pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(vector_cloudobject[1]);
		 viewer->addPointCloud<pcl::PointXYZRGB>(vector_cloudobject[1], rgb1, file2);
		 viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, file2);

		 if(vector_typeoject[1]==2||vector_typeoject[1]==3||vector_typeoject[1]==4||vector_typeoject[1]==5||vector_typeoject[1]==6
					 ||vector_typeoject[1]==12||vector_typeoject[1]==13||vector_typeoject[1]==14||vector_typeoject[1]==16
					 ||vector_typeoject[1]==17||vector_typeoject[1]==18||vector_typeoject[1]==19||vector_typeoject[1]==20)
		 {
			 viewer->addCylinder(*Vector_ceoficientobject[1],filecylin2,0);
			 viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,filecylin2);
		     viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,array_color[1].r,array_color[1].g,array_color[1].b,filecylin2,0);
			 if(vector_typeoject[1]==12)
			  {
				  sprintf(filetex2,"Paper Cup (%5.2f, %5.2f, %5.2f), Radius: %5.2f(m)\n", Vector_ceoficientobject[1]->values[0], Vector_ceoficientobject[1]->values[1],Vector_ceoficientobject[1]->values[2],Vector_ceoficientobject[1]->values[6]);
			  }
			  else if(vector_typeoject[1]==13)
			  {
				  sprintf(filetex2,"Medicine Vial (%5.2f, %5.2f, %5.2f), Radius: %5.2f(m)\n", Vector_ceoficientobject[1]->values[0], Vector_ceoficientobject[1]->values[1],Vector_ceoficientobject[1]->values[2],Vector_ceoficientobject[1]->values[6]);
			  }
			  else if(vector_typeoject[1]==14)
			  {
				  sprintf(filetex2,"Milk Bottle (%5.2f,%5.2f, %5.2f), Radius: %5.2f(m)\n", Vector_ceoficientobject[1]->values[0], Vector_ceoficientobject[1]->values[1],Vector_ceoficientobject[1]->values[2],Vector_ceoficientobject[1]->values[6]);
			  }
		 }
		 
		 viewer->addText(filetex2,400,145,30,0.0,0.0,0.0,filelocation2);
		 ///////////////////////////////////////////////////
		 pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(vector_cloudobject[2]);
		 viewer->addPointCloud<pcl::PointXYZRGB>(vector_cloudobject[2], rgb2, file3);
		 viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, file3);


		 if(vector_typeoject[2]==2||vector_typeoject[2]==3||vector_typeoject[2]==4||vector_typeoject[2]==5||vector_typeoject[2]==6
					 ||vector_typeoject[2]==12||vector_typeoject[2]==13||vector_typeoject[2]==14||vector_typeoject[2]==16
					 ||vector_typeoject[2]==17||vector_typeoject[2]==18||vector_typeoject[2]==19||vector_typeoject[2]==20)
		 {
			 viewer->addCylinder(*Vector_ceoficientobject[2],filecylin3,0);
			 viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,filecylin3);
		     viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,array_color[2].r,array_color[2].g,array_color[2].b,filecylin3,0);
			 if(vector_typeoject[2]==12)
			  {
				  sprintf(filetex3,"Paper Cup (%5.2f, %5.2f, %5.2f), Radius: %5.2f(m)\n", Vector_ceoficientobject[2]->values[0], Vector_ceoficientobject[2]->values[1],Vector_ceoficientobject[2]->values[2],Vector_ceoficientobject[2]->values[6]);
			  }
			  else if(vector_typeoject[2]==13)
			  {
				  sprintf(filetex3,"Medicine Vial (%5.2f, %5.2f, %5.2f), Radius: %5.2f(m)\n", Vector_ceoficientobject[2]->values[0], Vector_ceoficientobject[2]->values[1],Vector_ceoficientobject[2]->values[2],Vector_ceoficientobject[2]->values[6]);
			  }
			  else if(vector_typeoject[2]==14)
			  {
				  sprintf(filetex3,"Milk Bottle (%5.2f,%5.2f, %5.2f), Radius: %5.2f(m)\n", Vector_ceoficientobject[2]->values[0], Vector_ceoficientobject[2]->values[1],Vector_ceoficientobject[2]->values[2],Vector_ceoficientobject[2]->values[6]);
			  }
		 }
		 
		 viewer->addText(filetex3,400,190,30,0.0,0.0,0.0,filelocation3);
	 }
	  int time=0;
	 viewer->addCoordinateSystem (0.5);
	 viewer->setBackgroundColor (1.0, 1.0, 1.0, v1);
	 char file_save[500];
	  while(!viewer->wasStopped())
	  {
		  viewer->spinOnce (0);
		  time = clock();
		  if(time % 200==0)
		  {
			  sprintf(file_save, "G:/3D_object_DATASET/visyally_imapared_24_5/nam/testdata/cap2/Result_cylindrical_new/image_cloud_estimate55/timefitting_%d.png", time);
			  viewer->saveScreenshot(file_save);
		  }
	  }
	  /*if(frame==startframe)
	  {
		  viewer->spinOnce (5000);
	  }
	  else
	  {
		  viewer->spinOnce (0);
	  }*/
	//}
	 //viewer->saveScreenshot(fileimagewrite);
		  //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
}

void visualization_scene::visuali_three(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1, 
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2,int frame)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->removeAllPointClouds();
	 viewer->removeAllShapes();
	 viewer->removeCoordinateSystem();
	 viewer->removeCorrespondences();
	char filescene[100],file[100];
	 sprintf(filescene,"scene(%d)",frame);
	 sprintf(file,"tablescene(%d)",frame);
	 int v1(0);
	 stringstream sstm;
	 sstm << "Frame:  "<<frame;
	 viewer->setBackgroundColor (0, 0, 0, v1);
	 viewer->addText(sstm.str(), 10, 10, "v1 text", v1);
   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255,0, 0);
 viewer->addPointCloud<pcl::PointXYZ> (cloud ,single_color, filescene);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, filescene);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cloud1, 0,255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud1 ,single_color1, file);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, file);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud2, 0,0, 255);
  viewer->addPointCloud<pcl::PointXYZ> (cloud2 ,single_color2, file);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, file);

  viewer->addCoordinateSystem (1.0);

  /*while(!viewer->wasStopped())
  {*/
  if (frame==1)
  {
	   viewer->spinOnce (50000);
  }
  else
  {
	  viewer->spinOnce (0);
  }
	 
	  //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 //}

}

void visualization_scene::visuali_one(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int frame)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->removeAllPointClouds();
	 viewer->removeAllShapes();
	 viewer->removeCoordinateSystem();
	 viewer->removeCorrespondences();

	  char filescene[100];
	 sprintf(filescene,"scene(%d)",frame);

	 char filenormal[100];
	 sprintf(filenormal,"normal(%d)",frame);

	 int v1(0);
	 stringstream sstm;
	 sstm << "Frame:  "<<frame;
	 viewer->setBackgroundColor (0, 0, 0, v1);
	 viewer->addText(sstm.str(), 10, 10, "v1 text", v1);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255,0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud ,single_color, filescene);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, filescene);

  char filecup[100], filebow[100];
  //viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, cloud_normal, 1, 1, filenormal, v1);

  viewer->addCoordinateSystem (0.5);
  //viewer->initCameraParameters ();
  int flag=0;
  while(!viewer->wasStopped())
  {
	  
	  if (flag==1)
	  {
		  break;
	  }
	   viewer->spinOnce (10);
	  cout << '\n' << "Press a key to continue...";
	  system("pause");
	  char key =  getchar();
		if(key == 'c')
		{
			sprintf(filecup, "D:/ThuCSDLfix1/Kinect_data_2312/ob1/data_cylinder_bolw/cup/object_%d_3.pcd",frame);
			pcl::io::savePCDFile(filecup,*cloud);
			flag=1;
			break;
		}
		else if(key == 'b')
		{
			sprintf(filebow, "D:/ThuCSDLfix1/Kinect_data_2312/ob1/data_cylinder_bolw/bow/object_%d_3.pcd",frame);
			pcl::io::savePCDFile(filebow,*cloud);
			flag=1;
			break;
		}
		else if(key == 'n')
		{
			flag=1;
			break;
		}
  }
}


void visualization_scene::visuali_two_color_only(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,int frame)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->removeAllPointClouds();
	 viewer->removeAllShapes();
	 viewer->removeCoordinateSystem();
	 viewer->removeCorrespondences();
	viewer->removeCoordinateSystem();
	viewer->removeText3D();
	
	 int v1(0);
	/* stringstream sstm;
	 sstm << "Frame:  "<<frame;*/
	 viewer->setBackgroundColor (0, 0, 0, v1);
	 //viewer->addText(sstm.str(), 10, 10, "v1 text", v1);
	char filescene[100],file[100];
	sprintf(filescene,"scene(%d)",frame);

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, filescene);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, filescene);

  //viewer->addCoordinateSystem (2.0);
  /*char fileimage[100];
  sprintf(fileimage,"D:/ThuCSDLfix1/Kinect_data_2312/ob1/Image_output1/Image_ob1_%d.png", frame);*/
  viewer->setBackgroundColor(1.0,1.0,1.0);
  while(!viewer->wasStopped())
  {
  if(frame==81)
  {
	   viewer->spinOnce (10000);
  }
  else
  {
		viewer->spinOnce (0);
  }
	  
  //viewer->saveScreenshot(fileimage);
	  //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

}

void visualization_scene::visuali_cylinder(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normal,pcl::ModelCoefficients &cylinder_coefficitent,int frame)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->removeAllPointClouds();
	 viewer->removeAllShapes();
	 viewer->removeCoordinateSystem();
	 viewer->removeCorrespondences();
	viewer->removeCoordinateSystem();
	viewer->removeText3D();
	
	 int v1(0);
	/* stringstream sstm;
	 sstm << "Frame:  "<<frame;*/
	 viewer->setBackgroundColor (0, 0, 0, v1);

	 
	 /*viewer->removeShape("Object");
	 viewer->addText("Object 1\n",10,15,15,1.0,1.0,1.0,"Object");

	 char fileground[100];
	 sprintf(fileground,"Ground-truth: X= %f, Y= %f, Z= %f\n", cloud_ground->points[0].x, cloud_ground->points[0].y, cloud_ground->points[0].z);
	 viewer->removeShape("GroundTruth");
	 viewer->addText(fileground,10,30,15,1.0,1.0,1.0,"GroundTruth");

	 char filelocation[100];
	 sprintf(filelocation,"Estimated location: X= %f, Y= 0, Z= %f\n", cloud_location->points[0].x, cloud_location->points[0].z);
	 viewer->removeShape("Location");
	 viewer->addText(filelocation,10,45,15,1.0,1.0,1.0,"Location");
	
	 char fileradius[100];
	 sprintf(fileradius,"Radius of estemated object: R = %f cm\n", radius);
	 viewer->removeShape("EstimatedRadius");
	 viewer->addText(fileradius,10,60,15,1.0,1.0,1.0,"EstimatedRadius");

	 char filerror[100];
	 sprintf(filerror,"Error of object location: E = %f cm\n", err);
	 viewer->removeShape("ErrorLocation");
	 viewer->addText(filerror,10,75,15,1.0,1.0,1.0,"ErrorLocation");*/


	 //viewer->addText(sstm.str(), 10, 10, "v1 text", v1);
	char filescene[100],file[100];
	sprintf(filescene,"scene(%d)",frame);

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, filescene);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, filescene);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cloud1, 0,255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud1 ,single_color1, file);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,10, file);
  
  
  /*
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud_ground, 255,255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud_ground ,single_color2, "groundtruth");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,10, "groundtruth");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(cloud_location, 255,0, 255);
  viewer->addPointCloud<pcl::PointXYZ> (cloud_location ,single_color3, "location");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,10, "location");*/


    /*pcl::ModelCoefficients cylinder_coeff;
	cylinder_coeff.values.resize (7);   */ // We need 7 values
	//cylinder_coeff.values[0] = -2.6363;//cylinder_coefficitent->values[0];
	//cylinder_coeff.values[1] = -radius*10;//cylinder_coefficitent->values[1];
	//cylinder_coeff.values[2] = -11.3079;//cylinder_coefficitent->values[2];
	//cylinder_coeff.values[3] = 0.3093;//cylinder_coefficitent->values[3];
	//cylinder_coeff.values[4] = -0.9509;//cylinder_coefficitent->values[4];
	//cylinder_coeff.values[5] = 0.0066;//cylinder_coefficitent->values[5];
	//cylinder_coeff.values[6] = 0.295;//cylinder_coefficitent->values[6];

	//cylinder_coeff.values[0] = -3.05226;//cylinder_coefficitent->values[0];
	//cylinder_coeff.values[1] = 2.015;//cylinder_coefficitent->values[1];
	//cylinder_coeff.values[2] = -11.199;//cylinder_coefficitent->values[2];
	//cylinder_coeff.values[3] = -0.1399;//cylinder_coefficitent->values[3];
	//cylinder_coeff.values[4] = 0.9896;//cylinder_coefficitent->values[4];
	//cylinder_coeff.values[5] = 0.03215;//cylinder_coefficitent->values[5];
	//cylinder_coeff.values[6] = 0.3014;//cylinder_coefficitent->values[6];



	/*Eigen::Vector3f n1(cylinder_coeff.values[0], cylinder_coeff.values[1], cylinder_coeff.values[2]);
	Eigen::Vector3f n2(0, 1, 0);		
	double angle = pcl::rad2deg(acos(n1.dot(n2)));*/




      /*sprintf (name, "normal_%d", unsigned (i));
		viewer->addArrow (pt2, pt1, 1.0, 0, 0, false, name);*/
		
		viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud1, cloud_normal, 10, 1, "file normal", 0);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "file normal");

		viewer->addCylinder(cylinder_coefficitent,"cylinder_moi",0);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1.0,0.0,0.0,"cylinder_moi",0);


  viewer->addCoordinateSystem (2.0);
  char fileimage[100];
  sprintf(fileimage,"E:/LVH/AdaptiveRANSAC_clean/AdaptiveRANSAC_clean/Fitting_cylinder/output_fit/image_out/level_%d.png", frame);
  while(!viewer->wasStopped())
  {
  if(frame==81)
  {
	   viewer->spinOnce (10000);
  }
  else
  {
		viewer->spinOnce (0);
  }
	  
  viewer->saveScreenshot(fileimage);

	  //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

}

void visualization_scene::visuali_cylinderxyz(int method_d,int model_type,boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_r,
	pcl::ModelCoefficients &coefficitent_model,int frame)
{
  
    viewer->removeAllPointClouds();
	viewer->removeAllShapes();
	viewer->removeCoordinateSystem();
	viewer->removeCorrespondences();
	viewer->removeCoordinateSystem();
	viewer->removeText3D();
	int v1(0);
	viewer->setBackgroundColor (1, 1, 1, v1);
	
	char filescene[100],filedata1[100],filedata2[100],filedata3[100],filedata_r[100], fileselect[100], fileaxis[100], filecylinder[100],filesphere[100], filecone[100];
	sprintf(filescene,"scene(%d)",frame);

	sprintf(filedata1,"scene1(%d)",frame);
	sprintf(filedata2,"scene2(%d)",frame);
	sprintf(filedata3,"scene3(%d)",frame);

	sprintf(fileaxis,"fileaxis(%d)",frame);
	sprintf(filedata_r,"scene_r(%d)",frame);

	sprintf(fileselect,"scselect(%d)",frame);

	
    sprintf(filecylinder,"cylinder(%d)",frame);
	sprintf(filesphere,"sphere(%d)",frame);
	sprintf(filecone,"cone(%d)",frame);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255,0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud,single_color, filedata1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,5, filedata1);

	///*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_r(cloud_r, 0,0, 255);
	//viewer->addPointCloud<pcl::PointXYZ> (cloud_r,single_color_r, filedata_r);
 //   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,5, filedata_r);*/

	//draw line 

	pcl::PointXYZ newpoint1;
	newpoint1.x=coefficitent_model.values[0];
	newpoint1.y=coefficitent_model.values[1];
	newpoint1.z=coefficitent_model.values[2];
	
	pcl::PointXYZ newpoint2;
	newpoint2.x=coefficitent_model.values[3];
	newpoint2.y=coefficitent_model.values[4];
	newpoint2.z=coefficitent_model.values[5];

	



	viewer->addLine(newpoint1,newpoint2,"linecylinder",0);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.0,1.0,0.0,filecylinder,0);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,50,"linecylinder",0);




	 if (model_type==3)
	{
		viewer->addCylinder(coefficitent_model,filecylinder,0);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.0,1.0,0.0,filecylinder,0);
	}
	else if (model_type==4)
	{
		viewer->addSphere(coefficitent_model,filesphere,0);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.0,1.0,0.0,filesphere,0);
	}
	else if (model_type==5)
	{
		viewer->addCone(coefficitent_model,filecone,0);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.0,1.0,0.0,filecone,0);
	}

 viewer->addCoordinateSystem (0.1);
 /*char fileimage[100];
 if (method_d==1)
 {
	 sprintf(fileimage,"D:/ThuCSDLfix1/Kinect2.1/OSDdepth/result_image/source/frame_source_%d_object_%d.png", frame,index);
 }
 else if(method_d==2)
 {
	 sprintf(fileimage,"D:/ThuCSDLfix1/Kinect2.1/OSDdepth/result_image/impro/frame_impro_%d_object_%d.png", frame,index);
 }*/
 
	  while(!viewer->wasStopped())
	  {
		   viewer->spinOnce (0);
	 }
	 //viewer->saveScreenshot(fileimage); 
}




/////////////////////////////////////////////////////////////////////////////////////////////

	void visualization_scene::visuali_pointcloud_keypoint(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, 
		pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_scene,
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_keypoint, int frame, int numberdata,int framestart)
	{

		viewer->removeAllPointClouds();
		viewer->removeAllShapes();
		viewer->removeCoordinateSystem();
		viewer->removeCorrespondences();
		viewer->removeCoordinateSystem();
		viewer->removeText3D();
		int v1(0);
		viewer->setBackgroundColor(1, 1, 1, v1);

		char filescene[100];
		
		char filesphere1[100], filesphere2[100], filesphere3[100], filesphere4[100], filesphere5[100];
		char filesphere6[100], filesphere7[100], filesphere8[100], filesphere9[100], filesphere10[100];
		char filesphere11[100], filesphere12[100], filesphere13[100], filesphere14[100], filesphere15[100];
		char filesphere16[100], filesphere17[100], filesphere18[100], filesphere19[100], filesphere20[100];
		char filesphere21[100], filesphere22[100], filesphere23[100], filesphere24[100], filesphere25[100];


		char filesline1[100], filesline2[100], filesline3[100], filesline4[100], filesline5[100];
		char filesline6[100], filesline7[100], filesline8[100], filesline9[100], filesline10[100];
		char filesline11[100], filesline12[100], filesline13[100], filesline14[100], filesline15[100];
		char filesline16[100], filesline17[100], filesline18[100], filesline19[100], filesline20[100];

		sprintf(filescene, "scene(%d)", frame);

		sprintf(filesphere1, "sphere1");
		sprintf(filesphere2, "sphere2");
		sprintf(filesphere3, "sphere3");
		sprintf(filesphere4, "sphere4");
		sprintf(filesphere5, "sphere5");
		sprintf(filesphere6, "sphere6");
		sprintf(filesphere7, "sphere7");
		sprintf(filesphere8, "sphere8");
		sprintf(filesphere9, "sphere9");
		sprintf(filesphere10, "sphere10");
		sprintf(filesphere11, "sphere11");
		sprintf(filesphere12, "sphere12");
		sprintf(filesphere13, "sphere13");
		sprintf(filesphere14, "sphere14");
		sprintf(filesphere15, "sphere15");
		sprintf(filesphere16, "sphere16");
		sprintf(filesphere17, "sphere17");
		sprintf(filesphere18, "sphere18");
		sprintf(filesphere19, "sphere19");
		sprintf(filesphere20, "sphere20");
		sprintf(filesphere21, "sphere21");
		sprintf(filesphere22, "sphere22");
		sprintf(filesphere23, "sphere23");
		sprintf(filesphere24, "sphere24");
		sprintf(filesphere25, "sphere25");


		sprintf(filesline1, "line1");
		sprintf(filesline2, "line2");
		sprintf(filesline3, "line3");
		sprintf(filesline4, "line4");
		sprintf(filesline5, "line5");
		sprintf(filesline6, "line6");
		sprintf(filesline7, "line7");
		sprintf(filesline8, "line8");
		sprintf(filesline9, "line9");
		sprintf(filesline10, "line10");
		sprintf(filesline11, "line11");
		sprintf(filesline12, "line12");
		sprintf(filesline13, "line13");
		sprintf(filesline14, "line14");
		sprintf(filesline15, "line15");
		sprintf(filesline16, "line16");
		sprintf(filesline17, "line17");
		sprintf(filesline18, "line18");
		sprintf(filesline19, "line19");
		sprintf(filesline20, "line20");

		char fileimage[200];
		sprintf(fileimage, "G:/PROJECT_CODE/dulieu_ALL/Du_lieu_color_depth/l%d/pointcloud/frame_%d.png", numberdata, frame);


		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_scene);
		viewer->addPointCloud<pcl::PointXYZRGB>(cloud_scene, rgb, filescene);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, filescene);

		
		   //Nose/////////////////////////////////////////////////////////////////////////
			pcl::ModelCoefficients sphere_coeff1;
			sphere_coeff1.values.resize(4);
			// We need 7 values
			sphere_coeff1.values[0] = cloud_keypoint->points[0].x;
			sphere_coeff1.values[1] = cloud_keypoint->points[0].y;
			sphere_coeff1.values[2] = cloud_keypoint->points[0].z;
			sphere_coeff1.values[3] = 0.04;

			pcl::PointXYZ newpoint1;
			newpoint1.x = cloud_keypoint->points[0].x;
			newpoint1.y = cloud_keypoint->points[0].y;
			newpoint1.z = cloud_keypoint->points[0].z;
			if (newpoint1.x != 0 && newpoint1.y != 0 && newpoint1.z != 0.0)
			{
				viewer->addSphere(sphere_coeff1, filesphere1, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, filesphere1, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 0.0, 1.0, 0.0, filesphere1, 0);
				viewer->setRepresentationToSurfaceForAllActors();
			}

			//Neck///////////////////////////////////////////////////////////////////////////////
			pcl::ModelCoefficients sphere_coeff2;
			sphere_coeff2.values.resize(4);
			// We need 7 values
			sphere_coeff2.values[0] = cloud_keypoint->points[1].x;
			sphere_coeff2.values[1] = cloud_keypoint->points[1].y;
			sphere_coeff2.values[2] = cloud_keypoint->points[1].z;
			sphere_coeff2.values[3] = 0.04;

			pcl::PointXYZ newpoint2;
			newpoint2.x = cloud_keypoint->points[1].x;
			newpoint2.y = cloud_keypoint->points[1].y;
			newpoint2.z = cloud_keypoint->points[1].z;
			
			
			//R_Shoulder
			pcl::ModelCoefficients sphere_coeff3;
			sphere_coeff3.values.resize(4);
			// We need 7 values
			sphere_coeff3.values[0] = cloud_keypoint->points[2].x;
			sphere_coeff3.values[1] = cloud_keypoint->points[2].y;
			sphere_coeff3.values[2] = cloud_keypoint->points[2].z;
			sphere_coeff3.values[3] = 0.04;

			pcl::PointXYZ newpoint3;
			newpoint3.x = cloud_keypoint->points[2].x;
			newpoint3.y = cloud_keypoint->points[2].y;
			newpoint3.z = cloud_keypoint->points[2].z;
			
			//R_Elbow
			pcl::ModelCoefficients sphere_coeff4;
			sphere_coeff4.values.resize(4);
			// We need 7 values
			sphere_coeff4.values[0] = cloud_keypoint->points[3].x;
			sphere_coeff4.values[1] = cloud_keypoint->points[3].y;
			sphere_coeff4.values[2] = cloud_keypoint->points[3].z;
			sphere_coeff4.values[3] = 0.04;

			pcl::PointXYZ newpoint4;
			newpoint4.x = cloud_keypoint->points[3].x;
			newpoint4.y = cloud_keypoint->points[3].y;
			newpoint4.z = cloud_keypoint->points[3].z;
			
			//R_Wrist///////////////////////////////////////////
			pcl::ModelCoefficients sphere_coeff5;
			sphere_coeff5.values.resize(4);
			// We need 7 values
			sphere_coeff5.values[0] = cloud_keypoint->points[4].x;
			sphere_coeff5.values[1] = cloud_keypoint->points[4].y;
			sphere_coeff5.values[2] = cloud_keypoint->points[4].z;
			sphere_coeff5.values[3] = 0.04;

			pcl::PointXYZ newpoint5;
			newpoint5.x = cloud_keypoint->points[4].x;
			newpoint5.y = cloud_keypoint->points[4].y;
			newpoint5.z = cloud_keypoint->points[4].z;
			
			//LShoulder////////////////////////////////////////////////
			pcl::ModelCoefficients sphere_coeff6;
			sphere_coeff6.values.resize(4);
			// We need 7 values
			sphere_coeff6.values[0] = cloud_keypoint->points[5].x;
			sphere_coeff6.values[1] = cloud_keypoint->points[5].y;
			sphere_coeff6.values[2] = cloud_keypoint->points[5].z;
			sphere_coeff6.values[3] = 0.04;

			pcl::PointXYZ newpoint6;
			newpoint6.x = cloud_keypoint->points[5].x;
			newpoint6.y = cloud_keypoint->points[5].y;
			newpoint6.z = cloud_keypoint->points[5].z;
			if (newpoint6.x != 0 && newpoint6.y != 0 && newpoint6.z != 0.0)
			{
				viewer->addSphere(sphere_coeff6, filesphere6, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, filesphere6, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 0.0, 1.0, 0.0, filesphere6, 0);
				viewer->setRepresentationToSurfaceForAllActors();
			}
			//L_Elbow/////////////////////////////////////////////////////
			pcl::ModelCoefficients sphere_coeff7;
			sphere_coeff7.values.resize(4);
			// We need 7 values
			sphere_coeff7.values[0] = cloud_keypoint->points[6].x;
			sphere_coeff7.values[1] = cloud_keypoint->points[6].y;
			sphere_coeff7.values[2] = cloud_keypoint->points[6].z;
			sphere_coeff7.values[3] = 0.04;

			pcl::PointXYZ newpoint7;
			newpoint7.x = cloud_keypoint->points[6].x;
			newpoint7.y = cloud_keypoint->points[6].y;
			newpoint7.z = cloud_keypoint->points[6].z;
			
			//LWrist
			pcl::ModelCoefficients sphere_coeff8;
			sphere_coeff8.values.resize(4);
			// We need 7 values
			sphere_coeff8.values[0] = cloud_keypoint->points[7].x;
			sphere_coeff8.values[1] = cloud_keypoint->points[7].y;
			sphere_coeff8.values[2] = cloud_keypoint->points[7].z;
			sphere_coeff8.values[3] = 0.04;

			pcl::PointXYZ newpoint8;
			newpoint8.x = cloud_keypoint->points[7].x;
			newpoint8.y = cloud_keypoint->points[7].y;
			newpoint8.z = cloud_keypoint->points[7].z;
			
			//Mid_Hip
			pcl::ModelCoefficients sphere_coeff9;
			sphere_coeff9.values.resize(4);
			// We need 7 values
			sphere_coeff9.values[0] = cloud_keypoint->points[8].x;
			sphere_coeff9.values[1] = cloud_keypoint->points[8].y;
			sphere_coeff9.values[2] = cloud_keypoint->points[8].z;
			sphere_coeff9.values[3] = 0.04;

			pcl::PointXYZ newpoint9;
			newpoint9.x = cloud_keypoint->points[8].x;
			newpoint9.y = cloud_keypoint->points[8].y;
			newpoint9.z = cloud_keypoint->points[8].z;
			
			//R_Hip
			pcl::ModelCoefficients sphere_coeff10;
			sphere_coeff10.values.resize(4);
			// We need 7 values
			sphere_coeff10.values[0] = cloud_keypoint->points[9].x;
			sphere_coeff10.values[1] = cloud_keypoint->points[9].y;
			sphere_coeff10.values[2] = cloud_keypoint->points[9].z;
			sphere_coeff10.values[3] = 0.04;

			pcl::PointXYZ newpoint10;
			newpoint10.x = cloud_keypoint->points[9].x;
			newpoint10.y = cloud_keypoint->points[9].y;
			newpoint10.z = cloud_keypoint->points[9].z;
			

			//R_Knee////////////////////////////////////////////////////////////////////////////////////////
			pcl::ModelCoefficients sphere_coeff11;
			sphere_coeff11.values.resize(4);
			// We need 7 values
			sphere_coeff11.values[0] = cloud_keypoint->points[10].x;
			sphere_coeff11.values[1] = cloud_keypoint->points[10].y;
			sphere_coeff11.values[2] = cloud_keypoint->points[10].z;
			sphere_coeff11.values[3] = 0.04;

			pcl::PointXYZ newpoint11;
			newpoint11.x = cloud_keypoint->points[10].x;
			newpoint11.y = cloud_keypoint->points[10].y;
			newpoint11.z = cloud_keypoint->points[10].z;
			
			//R_Ankle//////////////////////////////////////////////////////////////////////////////////////
			pcl::ModelCoefficients sphere_coeff12;
			sphere_coeff12.values.resize(4);
			// We need 7 values
			sphere_coeff12.values[0] = cloud_keypoint->points[11].x;
			sphere_coeff12.values[1] = cloud_keypoint->points[11].y;
			sphere_coeff12.values[2] = cloud_keypoint->points[11].z;
			sphere_coeff12.values[3] = 0.04;

			pcl::PointXYZ newpoint12;
			newpoint12.x = cloud_keypoint->points[11].x;
			newpoint12.y = cloud_keypoint->points[11].y;
			newpoint12.z = cloud_keypoint->points[11].z;
			
			//L_Hip//////////////////////////////////////////////////////////////////////////////////////////////////////
			pcl::ModelCoefficients sphere_coeff13;
			sphere_coeff13.values.resize(4);
			// We need 7 values
			sphere_coeff13.values[0] = cloud_keypoint->points[12].x;
			sphere_coeff13.values[1] = cloud_keypoint->points[12].y;
			sphere_coeff13.values[2] = cloud_keypoint->points[12].z;
			sphere_coeff13.values[3] = 0.04;

			pcl::PointXYZ newpoint13;
			newpoint13.x = cloud_keypoint->points[12].x;
			newpoint13.y = cloud_keypoint->points[12].y;
			newpoint13.z = cloud_keypoint->points[12].z;
			
			//L_Knee
			pcl::ModelCoefficients sphere_coeff14;
			sphere_coeff14.values.resize(4);
			// We need 7 values
			sphere_coeff14.values[0] = cloud_keypoint->points[13].x;
			sphere_coeff14.values[1] = cloud_keypoint->points[13].y;
			sphere_coeff14.values[2] = cloud_keypoint->points[13].z;
			sphere_coeff14.values[3] = 0.04;

			pcl::PointXYZ newpoint14;
			newpoint14.x = cloud_keypoint->points[13].x;
			newpoint14.y = cloud_keypoint->points[13].y;
			newpoint14.z = cloud_keypoint->points[13].z;
			
			//L_Ankle/////////////////////////////////////////////////////////////////
			pcl::ModelCoefficients sphere_coeff15;
			sphere_coeff15.values.resize(4);
			// We need 7 values
			sphere_coeff15.values[0] = cloud_keypoint->points[14].x;
			sphere_coeff15.values[1] = cloud_keypoint->points[14].y;
			sphere_coeff15.values[2] = cloud_keypoint->points[14].z;
			sphere_coeff15.values[3] = 0.04;

			pcl::PointXYZ newpoint15;
			newpoint15.x = cloud_keypoint->points[14].x;
			newpoint15.y = cloud_keypoint->points[14].y;
			newpoint15.z = cloud_keypoint->points[14].z;
			if (newpoint15.x != 0 && newpoint15.y != 0 && newpoint15.z != 0.0)
			{
				viewer->addSphere(sphere_coeff15, filesphere15, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, filesphere15, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 0.0, 1.0, 0.0, filesphere15, 0);
				viewer->setRepresentationToSurfaceForAllActors();
				if (newpoint14.x != 0 && newpoint14.y != 0 && newpoint14.z != 0.0)
				{
					/*viewer->addLine(newpoint14, newpoint15, filesline13, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, filesline13, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, filesline13, 0);*/
				}
			}
			//R_Eye////////////////////////////////////////////////////////////////
			pcl::ModelCoefficients sphere_coeff16;
			sphere_coeff16.values.resize(4);
			// We need 7 values
			sphere_coeff16.values[0] = cloud_keypoint->points[15].x;
			sphere_coeff16.values[1] = cloud_keypoint->points[15].y;
			sphere_coeff16.values[2] = cloud_keypoint->points[15].z;
			sphere_coeff16.values[3] = 0.04;

			pcl::PointXYZ newpoint16;
			newpoint16.x = cloud_keypoint->points[15].x;
			newpoint16.y = cloud_keypoint->points[15].y;
			newpoint16.z = cloud_keypoint->points[15].z;

			//L_Eye
			pcl::ModelCoefficients sphere_coeff17;
			sphere_coeff17.values.resize(4);
			// We need 7 values
			sphere_coeff17.values[0] = cloud_keypoint->points[16].x;
			sphere_coeff17.values[1] = cloud_keypoint->points[16].y;
			sphere_coeff17.values[2] = cloud_keypoint->points[16].z;
			sphere_coeff17.values[3] = 0.04;

			pcl::PointXYZ newpoint17;
			newpoint17.x = cloud_keypoint->points[16].x;
			newpoint17.y = cloud_keypoint->points[16].y;
			newpoint17.z = cloud_keypoint->points[16].z;

			//R_Ear///////////////////////////////////////////////////////////////
			pcl::ModelCoefficients sphere_coeff18;
			sphere_coeff18.values.resize(4);
			// We need 7 values
			sphere_coeff18.values[0] = cloud_keypoint->points[17].x;
			sphere_coeff18.values[1] = cloud_keypoint->points[17].y;
			sphere_coeff18.values[2] = cloud_keypoint->points[17].z;
			sphere_coeff18.values[3] = 0.04;

			pcl::PointXYZ newpoint18;
			newpoint18.x = cloud_keypoint->points[17].x;
			newpoint18.y = cloud_keypoint->points[17].y;
			newpoint18.z = cloud_keypoint->points[17].z;

			//L_Ear
			pcl::ModelCoefficients sphere_coeff19;
			sphere_coeff19.values.resize(4);
			// We need 7 values
			sphere_coeff19.values[0] = cloud_keypoint->points[18].x;
			sphere_coeff19.values[1] = cloud_keypoint->points[18].y;
			sphere_coeff19.values[2] = cloud_keypoint->points[18].z;
			sphere_coeff19.values[3] = 0.04;

			pcl::PointXYZ newpoint19;
			newpoint19.x = cloud_keypoint->points[18].x;
			newpoint19.y = cloud_keypoint->points[18].y;
			newpoint19.z = cloud_keypoint->points[18].z;

			//L_BigToe/////////////////////////////////////////////////////////////////
			pcl::ModelCoefficients sphere_coeff20;
			sphere_coeff20.values.resize(4);
			// We need 7 values
			sphere_coeff20.values[0] = cloud_keypoint->points[19].x;
			sphere_coeff20.values[1] = cloud_keypoint->points[19].y;
			sphere_coeff20.values[2] = cloud_keypoint->points[19].z;
			sphere_coeff20.values[3] = 0.04;

			pcl::PointXYZ newpoint20;
			newpoint20.x = cloud_keypoint->points[19].x;
			newpoint20.y = cloud_keypoint->points[19].y;
			newpoint20.z = cloud_keypoint->points[19].z;
			
			//L_Small_Toe
			pcl::ModelCoefficients sphere_coeff21;
			sphere_coeff21.values.resize(4);
			// We need 7 values
			sphere_coeff21.values[0] = cloud_keypoint->points[20].x;
			sphere_coeff21.values[1] = cloud_keypoint->points[20].y;
			sphere_coeff21.values[2] = cloud_keypoint->points[20].z;
			sphere_coeff21.values[3] = 0.04;

			pcl::PointXYZ newpoint21;
			newpoint21.x = cloud_keypoint->points[20].x;
			newpoint21.y = cloud_keypoint->points[20].y;
			newpoint21.z = cloud_keypoint->points[20].z;

			//L_Heel
			pcl::ModelCoefficients sphere_coeff22;
			sphere_coeff22.values.resize(4);
			// We need 7 values
			sphere_coeff22.values[0] = cloud_keypoint->points[21].x;
			sphere_coeff22.values[1] = cloud_keypoint->points[21].y;
			sphere_coeff22.values[2] = cloud_keypoint->points[21].z;
			sphere_coeff22.values[3] = 0.04;

			pcl::PointXYZ newpoint22;
			newpoint22.x = cloud_keypoint->points[21].x;
			newpoint22.y = cloud_keypoint->points[21].y;
			newpoint22.z = cloud_keypoint->points[21].z;

			//R_BigToe
			pcl::ModelCoefficients sphere_coeff23;
			sphere_coeff23.values.resize(4);
			// We need 7 values
			sphere_coeff23.values[0] = cloud_keypoint->points[22].x;
			sphere_coeff23.values[1] = cloud_keypoint->points[22].y;
			sphere_coeff23.values[2] = cloud_keypoint->points[22].z;
			sphere_coeff23.values[3] = 0.04;
			
			pcl::PointXYZ newpoint23;
			newpoint23.x = cloud_keypoint->points[22].x;
			newpoint23.y = cloud_keypoint->points[22].y;
			newpoint23.z = cloud_keypoint->points[22].z;
			
			//R_Small_Toe
			pcl::ModelCoefficients sphere_coeff24;
			sphere_coeff24.values.resize(4);
			// We need 7 values
			sphere_coeff24.values[0] = cloud_keypoint->points[23].x;
			sphere_coeff24.values[1] = cloud_keypoint->points[23].y;
			sphere_coeff24.values[2] = cloud_keypoint->points[23].z;
			sphere_coeff24.values[3] = 0.04;

			pcl::PointXYZ newpoint24;
			newpoint24.x = cloud_keypoint->points[23].x;
			newpoint24.y = cloud_keypoint->points[23].y;
			newpoint24.z = cloud_keypoint->points[23].z;


			//R_Heel
			pcl::ModelCoefficients sphere_coeff25;
			sphere_coeff25.values.resize(4);
			// We need 7 values
			sphere_coeff25.values[0] = cloud_keypoint->points[24].x;
			sphere_coeff25.values[1] = cloud_keypoint->points[24].y;
			sphere_coeff25.values[2] = cloud_keypoint->points[24].z;
			sphere_coeff25.values[3] = 0.04;

			pcl::PointXYZ newpoint25;
			newpoint25.x = cloud_keypoint->points[24].x;
			newpoint25.y = cloud_keypoint->points[24].y;
			newpoint25.z = cloud_keypoint->points[24].z;

			//draw line ///////////////////////////////////////////////////////////////////////////////////////////////////////
			//2-9
			if (newpoint2.x != 0 && newpoint2.y != 0 && newpoint2.z != 0.0)
			{
				viewer->addSphere(sphere_coeff2, filesphere2, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, filesphere2, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 0.0, 1.0, 0.0, filesphere2, 0);
				viewer->setRepresentationToSurfaceForAllActors();
				if (newpoint9.x != 0 && newpoint9.y != 0 && newpoint9.z != 0.0)
				{
					/*viewer->addLine(newpoint2, newpoint9, filesline1, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, filesline1, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, filesline1, 0);*/
				}
			}
			////2-3

			if (newpoint3.x != 0 && newpoint3.y != 0 && newpoint3.z != 0.0)
			{
				viewer->addSphere(sphere_coeff3, filesphere3, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, filesphere3, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 0.0, 1.0, 0.0, filesphere3, 0);
				viewer->setRepresentationToSurfaceForAllActors();
				if (newpoint2.x != 0 && newpoint2.y != 0 && newpoint2.z != 0.0)
				{
					/*viewer->addLine(newpoint2, newpoint3, filesline2, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, filesline2, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, filesline2, 0);*/
				}

			}
			///3-4
			if (newpoint4.x != 0 && newpoint4.y != 0 && newpoint4.z != 0.0)
			{
				viewer->addSphere(sphere_coeff4, filesphere4, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, filesphere4, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 0.0, 1.0, 0.0, filesphere4, 0);
				viewer->setRepresentationToSurfaceForAllActors();
				if (newpoint3.x != 0 && newpoint3.y != 0 && newpoint3.z != 0.0)
				{
					/*viewer->addLine(newpoint3, newpoint4, filesline3, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, filesline3, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, filesline3, 0);*/
				}
			}

			//4-5
			if (newpoint5.x != 0 && newpoint5.y != 0 && newpoint5.z != 0.0)
			{
				viewer->addSphere(sphere_coeff5, filesphere5, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, filesphere5, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 0.0, 1.0, 0.0, filesphere5, 0);
				viewer->setRepresentationToSurfaceForAllActors();
				if (newpoint4.x != 0 && newpoint4.y != 0 && newpoint4.z != 0.0)
				{
					/*viewer->addLine(newpoint4, newpoint5, filesline4, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, filesline4, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, filesline4, 0);*/
				}
			}
			//6-7
			if (newpoint7.x != 0 && newpoint7.y != 0 && newpoint7.z != 0.0)
			{
				viewer->addSphere(sphere_coeff7, filesphere7, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, filesphere7, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 0.0, 1.0, 0.0, filesphere7, 0);
				viewer->setRepresentationToSurfaceForAllActors();
				if (newpoint6.x != 0 && newpoint6.y != 0 && newpoint6.z != 0.0)
				{
					/*viewer->addLine(newpoint6, newpoint7, filesline5, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, filesline5, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, filesline5, 0);*/
				}
			}

			//7-8
			if (newpoint8.x != 0 && newpoint8.y != 0 && newpoint8.z != 0.0)
			{
				viewer->addSphere(sphere_coeff8, filesphere8, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, filesphere8, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 0.0, 1.0, 0.0, filesphere8, 0);
				viewer->setRepresentationToSurfaceForAllActors();
				if (newpoint7.x != 0 && newpoint7.y != 0 && newpoint7.z != 0.0)
				{
					/*viewer->addLine(newpoint7, newpoint8, filesline6, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, filesline6, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, filesline6, 0);*/
				}
			}

			if (newpoint9.x != 0 && newpoint9.y != 0 && newpoint9.z != 0.0)
			{
				viewer->addSphere(sphere_coeff9, filesphere9, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, filesphere9, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 0.0, 1.0, 0.0, filesphere9, 0);
				viewer->setRepresentationToSurfaceForAllActors();
				if (newpoint2.x != 0 && newpoint2.y != 0 && newpoint2.z != 0.0)
				{
					/*viewer->addLine(newpoint2, newpoint9, filesline7, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, filesline7, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, filesline7, 0);*/
				}
			}

			//9-10
			if (newpoint10.x != 0 && newpoint10.y != 0 && newpoint10.z != 0.0)
			{
				viewer->addSphere(sphere_coeff10, filesphere10, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, filesphere10, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 0.0, 1.0, 0.0, filesphere10, 0);
				viewer->setRepresentationToSurfaceForAllActors();
				if (newpoint9.x != 0 && newpoint9.y != 0 && newpoint9.z != 0.0)
				{
					/*viewer->addLine(newpoint9, newpoint10, filesline8, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, filesline8, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, filesline8, 0);*/
				}
			}
			//10-11
			if (newpoint11.x != 0 && newpoint11.y != 0 && newpoint11.z != 0.0)
			{
				viewer->addSphere(sphere_coeff11, filesphere11, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, filesphere11, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 0.0, 1.0, 0.0, filesphere11, 0);
				viewer->setRepresentationToSurfaceForAllActors();
				if (newpoint10.x != 0 && newpoint10.y != 0 && newpoint10.z != 0.0)
				{
					/*viewer->addLine(newpoint10, newpoint11, filesline9, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, filesline9, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, filesline9, 0);*/
				}
			}
			//11-12
			if (newpoint12.x != 0 && newpoint12.y != 0 && newpoint12.z != 0.0)
			{
				viewer->addSphere(sphere_coeff12, filesphere12, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, filesphere12, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 0.0, 1.0, 0.0, filesphere12, 0);
				viewer->setRepresentationToSurfaceForAllActors();
				if (newpoint11.x != 0 && newpoint11.y != 0 && newpoint11.z != 0.0)
				{
					/*viewer->addLine(newpoint11, newpoint12, filesline10, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, filesline10, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, filesline10, 0);*/
				}
			}
			//9-13
			if (newpoint13.x != 0 && newpoint13.y != 0 && newpoint13.z != 0.0)
			{
				viewer->addSphere(sphere_coeff13, filesphere13, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, filesphere13, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 0.0, 1.0, 0.0, filesphere13, 0);
				viewer->setRepresentationToSurfaceForAllActors();
				if (newpoint9.x != 0 && newpoint9.y != 0 && newpoint9.z != 0.0)
				{
					/*viewer->addLine(newpoint9, newpoint13, filesline11, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, filesline11, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, filesline11, 0);*/
				}
			}
			//13-14
			if (newpoint14.x > 0 && newpoint14.y > 0 && newpoint14.z > 0.0)
			{
				viewer->addSphere(sphere_coeff14, filesphere14, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, filesphere14, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 0.0, 1.0, 0.0, filesphere14, 0);
				viewer->setRepresentationToSurfaceForAllActors();
				if (newpoint13.x != 0 && newpoint13.y != 0 && newpoint13.z != 0.0)
				{
					/*viewer->addLine(newpoint13, newpoint14, filesline12, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, filesline12, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, filesline12, 0);*/
				}
			}
			//14-15
			if (newpoint14.x > 0 && newpoint14.y > 0 && newpoint14.z > 0.0)
			{
				viewer->addSphere(sphere_coeff14, filesphere14, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, filesphere14, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 0.0, 1.0, 0.0, filesphere14, 0);
				viewer->setRepresentationToSurfaceForAllActors();
				if (newpoint15.x != 0 && newpoint15.y != 0 && newpoint15.z != 0.0)
				{
					/*viewer->addLine(newpoint14, newpoint15, filesline16, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, filesline16, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, filesline16, 0);*/
				}
			}

			//2-1
			if (newpoint2.x > 0 && newpoint2.y > 0 && newpoint2.z > 0.0)
			{
				viewer->addSphere(sphere_coeff12, filesphere2, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, filesphere2, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 0.0, 1.0, 0.0, filesphere2, 0);
				viewer->setRepresentationToSurfaceForAllActors();
				if (newpoint1.x != 0 && newpoint1.y != 0 && newpoint1.z != 0.0)
				{
					/*viewer->addLine(newpoint2, newpoint1, filesline17, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, filesline17, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, filesline17, 0);*/
				}
			}
			//1-16
			if (newpoint1.x > 0 && newpoint1.y > 0 && newpoint1.z > 0.0)
			{
				viewer->addSphere(sphere_coeff1, filesphere1, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, filesphere1, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 0.0, 1.0, 0.0, filesphere1, 0);
				viewer->setRepresentationToSurfaceForAllActors();
				if (newpoint16.x != 0 && newpoint16.y != 0 && newpoint16.z != 0.0)
				{
					/*viewer->addLine(newpoint1, newpoint16, filesline18, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, filesline18, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, filesline18, 0);*/
				}
			}
			//1-17
			if (newpoint1.x > 0 && newpoint1.y > 0 && newpoint1.z > 0.0)
			{
				viewer->addSphere(sphere_coeff1, filesphere1, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, filesphere1, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 0.0, 1.0, 0.0, filesphere1, 0);
				viewer->setRepresentationToSurfaceForAllActors();
				if (newpoint17.x != 0 && newpoint17.y != 0 && newpoint17.z != 0.0)
				{
					/*viewer->addLine(newpoint1, newpoint17, filesline19, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, filesline19, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, filesline19, 0);*/
				}
			}

			//15-20
			if (newpoint20.x != 0 && newpoint20.y != 0 && newpoint20.z != 0.0)
			{
				viewer->addSphere(sphere_coeff20, filesphere20, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, filesphere20, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 0.0, 1.0, 0.0, filesphere20, 0);
				viewer->setRepresentationToSurfaceForAllActors();
				if (newpoint15.x != 0 && newpoint15.y != 0 && newpoint15.z != 0.0)
				{
					/*viewer->addLine(newpoint15, newpoint20, filesline14, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, filesline14, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, filesline14, 0);*/
				}
			}
			//12-23
			if (newpoint23.x != 0 && newpoint23.y != 0 && newpoint23.z != 0.0)
			{
				viewer->addSphere(sphere_coeff23, filesphere23, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, filesphere23, 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 0.0, 1.0, 0.0, filesphere23, 0);
				viewer->setRepresentationToSurfaceForAllActors();
				if (newpoint12.x != 0 && newpoint12.y != 0 && newpoint12.z != 0.0)
				{
					/*viewer->addLine(newpoint12, newpoint23, filesline15, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, filesline15, 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, filesline15, 0);*/
				}
			}




		viewer->addCoordinateSystem(1.0);
		/*while (!viewer->wasStopped())
		{
			viewer->spinOnce(0);
		}*/

		if (frame == framestart)
		{
			viewer->spinOnce(10000);
		}
		else
		{
			viewer->spinOnce(0);
		}
		viewer->saveScreenshot(fileimage);



		//viewer->saveScreenshot(fileimage); 
	}