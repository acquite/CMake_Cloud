// https://blog.csdn.net/qq_34719188/article/details/79183358
// PCL 点云曲面重建与点云配准

// 最小二乘法对点云进行平滑处理
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>  //kd-tree搜索对象的类定义的头文件
#include <pcl/surface/mls.h>        //最小二乘法平滑处理类定义头文件

int test_mlsPCL8();


// 在平面模型上提取凸（凹）多边形

#include <pcl/ModelCoefficients.h>           //采样一致性模型相关类头文件
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>          //滤波相关类头文件
#include <pcl/segmentation/sac_segmentation.h>   //基于采样一致性分割类定义的头文件
#include <pcl/surface/concave_hull.h>                 //创建凹多边形类定义头文件

int test_PlanePCL8();



// 无序点云的快速三角化
// 使用贪婪投影三角化算法对有向点云进行三角化
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>      //贪婪投影三角化算法

int test_TripPCL8();


// PCL 点云配准
#include <pcl-1.8/pcl/registration/icp.h>   //ICP配准类相关头文件
int test_ICPPCL8();


// 逐步匹配多幅点云
#include <boost/make_shared.hpp>              //boost指针相关头文件
#include <pcl/point_types.h>                  //点类型定义头文件
#include <pcl/point_cloud.h>                  //点云类定义头文件
#include <pcl/point_representation.h>         //点表示相关的头文件
#include <pcl/filters/voxel_grid.h>           //用于体素网格化的滤波类头文件 
#include <pcl/filters/filter.h>             //滤波相关头文件
#include <pcl/features/normal_3d.h>         //法线特征头文件
#include <pcl/registration/icp.h>           //ICP类相关头文件
#include <pcl/registration/icp_nl.h>        //非线性ICP 相关头文件
#include <pcl/registration/transforms.h>      //变换矩阵类头文件
#include <pcl/visualization/pcl_visualizer.h>  //可视化类头文件

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

int test_ICPPcl8(int argc, char** argv);


// 如果观察不到结果，就按键R来重设摄像头，调整角度可以观察到有红绿两组点云显示在窗口的左边，红色为源点云，将看到上面的类似结果，命令行提示需要执行配准按下Q，按下后可以发现左边的窗口不断的调整点云，其实是配准过程中的迭代中间结果的输出，在迭代次数小于设定的次数之前，右边会不断刷新最新的配准结果，直到收敛，迭代次数30次完成整个匹配的过程，再次按下Q后会看到存储的1.pcd文件，此文件为第一个和第二个点云配准后与第一个输入点云在同一个坐标系下的点云。