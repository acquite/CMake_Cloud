// https://blog.csdn.net/qq_34719188/article/details/79181394

// PCL 采样一致性算法
// 随机采样一致性算法（RANSAC）
// 最大似然一致性算法（MLESAC）
// 最小中值方差一致性算法（LMEDS）

#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis_5 (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

/******************************************************************************************************************
 对点云进行初始化，并对其中一个点云填充点云数据作为处理前的的原始点云，其中大部分点云数据是基于设定的圆球和平面模型计算
  而得到的坐标值作为局内点，有1/5的点云数据是被随机放置的组委局外点。
 *****************************************************************************************************************/

int test_sacPCL5(int argc, char** argv);