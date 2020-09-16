//2020-09-14 Ray
#include <pcl/visualization/cloud_viewer.h>//类cloudviewer 头文件声名
#include<pcl/visualization/pcl_visualizer.h>
#include<iostream>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include<pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>


void viewerOneOff(pcl::visualization::PCLVisualizer& viewer);

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer);

void test1();

void test2();

void showHelp(char * program_name);

void test3(int argc, char ** argv);