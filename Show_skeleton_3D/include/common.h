/** @file common.h
 *  @brief Common function definitions for the different projects.
 *
 *
 *  @author Michiel Vlaminck
 *
 *	@date Nov 3, 2015
 *  
 *	@bug n/a
 */


void createDepthMap(cv::Mat &depthMap);

void createDepthMap(cv::Mat &raw_depth_map, cv::Mat &depth_map);

void createXYZRGBPointCloud1(cv::Mat& depth_image, cv::Mat& rgb_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud);
void createXYZRGBPointCloud2(cv::Mat& depth_image, cv::Mat& rgb_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud);

void createDepthImagefromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, cv::Mat& depthImage, cv::Rect &Object_Rect);


void createXYZRGBPointCloud_superpixel(cv::Mat& depthImage, cv::Mat& rgbImage, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud);

void createXYZRGBPointCloudUnregistered(cv::Mat& depthImage, cv::Mat& rgbImage, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& outputPointcloud);

void rotatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Eigen::Vector3f &normal);

void estimatePlaneParameters(pcl::PointIndices &inlier_indices, pcl::ModelCoefficients &coefficients);

void mergePlanes(std::vector<pcl::ModelCoefficients> &coefficients, std::vector<pcl::PointIndices> &inlier_indices);

void reflectPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

void createXYZRGBPointCloud_crop(int originx, int originy, int w, int h, cv::Mat& depthImage, cv::Mat& rgbImage, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud);

void ConvertInverXYZRGBtoRGB(cv::Mat& depthImage, cv::Mat& rgbImage, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& Input_cloud);

void createXYZRGBPointCloud_dowsampling331(cv::Mat& depthImage, cv::Mat& rgbImage, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud);

void createXYZRGBPointCloud_dowsampling331_crop(cv::Rect rect_object,cv::Mat& depthImage, cv::Mat& rgbImage, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud);

void createXYZRGBPointCloud_dowsampling332(cv::Mat& depthImage, cv::Mat& rgbImage, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud);

void createXYZPointCloud(cv::Mat& depthImage, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPointCloud);
void createXYZPointCloud_crop(int originx, int originy, int w, int h,cv::Mat& depthImage, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPointCloud);

void CreateXYZRGBtoRGB_EachObject(cv::Mat& depthImage_full,cv::Mat& depthImage_anotation, 
	cv::Mat& Color_img,  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& vec_out_cloudobject);

void CreateXYZRGBA_Point(cv::Mat& depthImage,cv::Mat& colorImage,int x,int y, pcl::PointCloud<pcl::PointXYZ>::Ptr& A_point);
void CreateXYZRGBTwo_Point(cv::Mat& depthImage,cv::Mat& colorImage,int x1, int y1, int x2, int y2, pcl::PointCloud<pcl::PointXYZ>::Ptr& Two_point);

void createXYZRGBPointCloud(cv::Mat& depthImage, cv::Mat& rgbImage, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud);
void createXYZRGBPointCloud_keypoint(cv::Mat& depthImage, cv::Mat& rgbImage, std::vector<Vector4> &vec_keypoint, pcl::PointCloud<pcl::PointXYZ>::Ptr &vec_cloud_keypoint);