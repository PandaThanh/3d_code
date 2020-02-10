#define _CRT_SECURE_NO_DEPRECATE  
// PCL include files
#include <pcl/correspondence.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include<pcl/io/pcd_io.h>
//
////
////
#include <pcl/features/normal_3d.h>

#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/cloud_iterator.h>

#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/common/io.h>
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/boost.h>
//
//#include <pcl/filters/passthrough.h>
#define _CRT_SECURE_NO_WARNINGS
  
// OpenCV include files
 #include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

//#include "opencv2/videostab.hpp"
#include"opencv2/contrib/contrib.hpp"
#include"opencv2/flann/lsh_table.h"

#include <stdlib.h>



#include <Windows.h>
#include <Ole2.h>

#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>
#include<NuiSkeleton.h>

#include <opencv2\opencv.hpp>
#include <sys/timeb.h>
#include <time.h>
#include <map>
#include <list>
// Other include files
#include <iostream>
#include <fstream>

//#include "common.h"
//#include"common.h"
//#include"segmentation.h"
//#include"normal_3d.h"


//#include"organized_multi_plane_segmentation.h"
//#include"organized_connected_component_segmentation.h"
//#include"Get_coor_crop.h"
#include"Get_coor_ACC.h"

#include<opencv/cv.h>
#include <limits>
#include<math.h>


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>
#include <iostream>
#include <vector>

#include "opencv/ml.h"
#include "opencv/cvaux.h"
#include "opencv/highgui.h"
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

 #include<iostream>
 #include "opencv2/imgproc/imgproc.hpp"
 #include "opencv2/highgui/highgui.hpp"
 #include <opencv2/core/core.hpp>
 //#include<dirent.h>
 #include<string.h>

using namespace std;
using namespace cv;
using namespace pcl;



