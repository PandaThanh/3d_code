#include"Get_coor_ACC.h"



void getcoorAccelerationVeector::getcooracc(char pathInput[1000],int row, int maxrow,  double matrix[4])
{
	int co=0;
	vector<string> QueryArray;
	ifstream qfile(pathInput);
	std::string line;
	if (co==0)
	{
		while (getline(qfile, line))
		{
			QueryArray.push_back(line);
		}
		co=1;
	}
	if (row<maxrow)
	 {
        std::istringstream stream(QueryArray[row-1]);
        double x;
        int col = 0;  // reset column counter
		
        while (stream >> x) 
		{  // keep trying to read ints until there are no more
            matrix[col] = x;
            col++;
	    }
		
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////
void getcoorAccelerationVeector::getKeypoint(char pathInput[1000], std::vector<Vector4> &vec_keypoint)
{
	int co = 0;
	vector<string> QueryArray;
	ifstream qfile(pathInput);
	std::string line;
	if (co == 0)
	{
		while (getline(qfile, line))
		{
			QueryArray.push_back(line);
		}
		co = 1;
	}
	for (int row=1; row<=QueryArray.size();row++)
	{
		std::istringstream stream(QueryArray[row - 1]);
		double x;
		int col = 0;  // reset column counter
		Vector4 each_keypoint;
		while (stream >> x)
		{  // keep trying to read ints until there are no more
			//matrix[col] = x;
			if (col == 0)
			{
				each_keypoint.w = x;
			}
			else if (col == 1)
			{
				each_keypoint.x = x;
			}
			else if (col == 2)
			{
				each_keypoint.y = x;
			}
			else if (col == 3)
			{
				each_keypoint.z = x;
			}
			col++;
		}
		vec_keypoint.push_back(each_keypoint);
	}
}



//////////////////////////////////////////////////////////////////////////////////


void getcoorAccelerationVeector::getcooracc_allrow(char pathInput[1000],int row, int maxrow, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_target3point)
{
	int co=0;
	vector<string> QueryArray;
	ifstream qfile(pathInput);
	std::string line;
	if (co==0)
	{
		while (getline(qfile, line))
		{
			QueryArray.push_back(line);
		}
		co=1;
	}
	cloud_target3point->resize(maxrow);
	if (row<maxrow)
	 {
		 for(int i=1;i<maxrow;i++)
		{
			std::istringstream stream(QueryArray[i-1]);
			double x;
			int col = 0;  // reset column counter
			while (stream >> x && col<3) 
			{  // keep trying to read ints until there are no more
				if(col==0)
				{
					cloud_target3point->points[i-1].x= x;
				}
				else if(col==1)
				{
					cloud_target3point->points[i-1].y= x;
				}
				else if(col==2)
				{
					cloud_target3point->points[i-1].z= x;
				}
				
				col++;
			}
		}
		
	
	}
}