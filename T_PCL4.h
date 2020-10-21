// https://blog.csdn.net/qq_34719188/article/details/79179430


// 直通滤波器对点云进行滤波处理
// 最简单的例子：比如说高程筛选
#include <iostream>
#include <pcl-1.8/pcl/point_types.h>
#include<pcl-1.8/pcl/filters/passthrough.h>

int test_mainPCL4(int argc, char** argv);


//VoxelGrid滤波器对点云进行下采样
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/filters/voxel_grid.h>
int test_VoxGridPCL4();


// statisticalOutlierRemoval滤波器移除离群点
#include <pcl-1.8/pcl/filters/statistical_outlier_removal.h>

int test_sorPCL4();

// 使用参数化模型投影点云
#include<pcl-1.8/pcl/filters/project_inliers.h>//投影滤波类头文件
#include<pcl-1.8/pcl/ModelCoefficients.h>//模型系数头文件

int test_proPCL4();

// 从一个点云中提取索引
// 基于某一分割算法提取点云中的一个子集
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

int test_segPCL4();

// ConditionalRemoval 或 RadiusOutlinerRemoval 移除离群点
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

int test_rcRemovalPCL4(int argc, char** argv);