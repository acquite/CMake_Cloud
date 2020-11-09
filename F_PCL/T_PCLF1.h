// https://blog.csdn.net/ys578676728/article/details/104657262
// 导入pcd文件
#include<pcl/io/pcd_io.h>//PCD读写类相关的头文件
#include<pcl/point_types.h>//PCL中支持的点类型的头文件

#include<iostream>
void test_readPCD_PCLF1();

//显示
#include<pcl/visualization/cloud_viewer.h>
void test_showCloud_PCLF1(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

// 移除地面