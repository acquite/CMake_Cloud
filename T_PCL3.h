//https://blog.csdn.net/qq_34719188/article/details/79179251

//可视化深度图像

#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


// --------------
// -----Help-----
// --------------
void printUsage (const char* progName);

void setViewerPose_3 (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose); //设置视角位置

// --------------
// -----Main-----
// --------------
int test_mainPCL3(int argc,char** argv);
