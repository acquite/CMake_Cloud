// https://blog.csdn.net/qq_34719188/article/details/79181704

// 获得深度图像
#include <pcl-1.8/pcl/range_image/range_image.h> //深度图像的头文件
void test_rImgPCL6();

// 有序点云数据深度图像
#include <boost/thread/thread.hpp>
#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/range_image/range_image.h>              //关于深度图像的头文件
#include <pcl/visualization/pcl_visualizer.h>         //PCL可视化的头文件
#include <pcl/visualization/range_image_visualizer.h> //深度图可视化的头文件

//命令帮助提示
void printUsage_6(const char *progName);

void setViewerPose_6(pcl::visualization::PCLVisualizer &viewer, const Eigen::Affine3f &viewer_pose);

int test_ordarRImgPCL6(int argc, char **argv);

//深度图像提取边界
#include <boost/thread/thread.hpp>
#include <iostream>
#include <pcl-1.8/pcl/console/parse.h>
#include <pcl-1.8/pcl/features/range_image_border_extractor.h>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/range_image/range_image.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.8/pcl/visualization/range_image_visualizer.h>

void printUsage_b6(const char *progName);
int test_borderPCL6(int argc, char **argv);

//关键点
#include <boost/thread/thread.hpp>
#include <iostream>
#include <pcl-1.8/pcl/range_image/range_image.h>
#include <pcl/console/parse.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>

void printUsage_narf(const char *progName);
void setViewerPose_narf(pcl::visualization::PCLVisualizer &viewer, const Eigen::Affine3f &viewer_pose); //设置视口的位姿

int test_narfPCL6(int argc, char **argv);
