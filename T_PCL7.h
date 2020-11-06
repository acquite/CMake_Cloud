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