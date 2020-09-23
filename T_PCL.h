//2020-09-14 Ray
#include <pcl/visualization/cloud_viewer.h> //类cloudviewer 头文件声名
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/passthrough.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/integral_image_normal.h>

void viewerOneOff(pcl::visualization::PCLVisualizer &viewer);

void viewerPsycho(pcl::visualization::PCLVisualizer &viewer);

void test1();

void test2();

void showHelp(char *program_name);

void test3(int argc, char **argv);

//PassThrough过滤【23】
void test_passThrough();

//IntegralImage积分图像【15】
void test_Integral();

#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/console/parse.h>

void setViewerPose(pcl::visualization::PCLVisualizer &viewer, const Eigen::Affine3f &viewer_pose);

//Narf 【19】
void test_Narf(int argc, char **argv);