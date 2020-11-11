// https://blog.csdn.net/qq_34719188/article/details/79183199
// PCL 点云特征描述与提取

// PCL 法线估计实例 ------ 估计某一点的表面法线
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>  //法线估计类头文件
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

void test_normalPCL7();

// PCL 法线估计实例 ------ 估计一个点云的表面法线
// PCL 使用积分图像进行法线估计--注意此方法只适用于有序点云
void test_normalOrderPCL7();

// PCL 使用积分图像进行法线估计
void test_IntegralPCL7();

// 点特征直方图（PFH）描述子
#include <pcl/features/pfh.h>                 
//pfh特征估计类头文件
// 快速点特征直方图(FPFH)描述子
// 视点特征直方图（或VFH）源于FPFH描述子
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>                     //VFH特征估计类头文件


// 从一个深度图像提取NARF特征
#include <boost/thread/thread.hpp>
#include <pcl-1.8/pcl/range_image/range_image.h>
#include <pcl-1.8/pcl/visualization/range_image_visualizer.h>
#include <pcl-1.8/pcl/features/range_image_border_extractor.h>
#include <pcl-1.8/pcl/keypoints/narf_keypoint.h>
#include <pcl-1.8/pcl/features/narf_descriptor.h>
#include <pcl-1.8/pcl/console/parse.h>

//命令帮助
void printUsage_7(const char *progName);

void setViewerPose_7(pcl::visualization::PCLVisualizer &viewer, const Eigen::Affine3f &viewer_pose); //setViewerPose

int test_NarfPCL7(int argc, char **argv);


// 特征描述算子算法基准化分析FeatureCorrespondenceTest