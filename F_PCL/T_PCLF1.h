// https://blog.csdn.net/ys578676728/article/details/104657262
// 导入pcd文件
#include<pcl-1.8/pcl/io/pcd_io.h>//PCD读写类相关的头文件
#include<pcl-1.8/pcl/point_types.h>//PCL中支持的点类型的头文件

#include<iostream>

//入口
void test_InitPCLF1();

//文件读取
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr test_readPCD_PCLF1();

//显示
#include<pcl-1.8/pcl/visualization/cloud_viewer.h>
void test_showCloud_PCLF1(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

// 移除地面
