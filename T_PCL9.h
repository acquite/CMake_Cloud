// PCL点云变换与移除NaN

#include <iostream>

#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/io/ply_io.h>


#include<pcl-1.8/pcl/point_cloud.h>
#include<pcl-1.8/pcl/console/parse.h>
#include<pcl-1.8/pcl/common/transforms.h>
#include<pcl-1.8/pcl/visualization/pcl_visualizer.h>

void showHelp_9(char * program_name);

int test_switchPCL9(int argc, char** argv);


// 移除 NaNs
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/cloud_viewer.h>

int test_nanPCL9(int argc,char** argv);