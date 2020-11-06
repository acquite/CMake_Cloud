#include "T_PCL7.h"

void test_normalPCL7()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // pcl::io::loadPCDFile("../data/table_scene_lms400.pcd", *cloud);
    pcl::io::loadPCDFile("../data/my_color.pcd", *cloud);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud < pcl::Normal>);
    ne.setRadiusSearch(0.1);
    ne.compute(*cloud_normals);

    //可视化
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.0);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals);

    //视点默认坐标是（0，0，0）可使用 setViewPoint(float vpx,float vpy,float vpz)来更换
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }

}

void test_normalOrderPCL7()
{
    //打开点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("../data/table_scene_mug_stereo_textured.pcd", *cloud);

    //创建法线估计向量
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

    /****************************************************************************************
         三种法线估计方法
          COVARIANCE_MATRIX 模式从具体某个点的局部邻域的协方差矩阵创建9个积分，来计算这个点的法线
          AVERAGE_3D_GRADIENT   模式创建6个积分图来计算水平方向和垂直方向的平滑后的三维梯度并使用两个梯度间的向量
                                积计算法线
          AVERAGE_DEPTH——CHANGE  模式只创建了一个单一的积分图，从而平局深度变化计算法线
    ********************************************************************************************/
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);  //设置法线估计的方式AVERAGE_3D_GRADIENT

    ne.setMaxDepthChangeFactor(0.02f);   //设置深度变化系数
    ne.setNormalSmoothingSize(10.0f);   //设置法线优化时考虑的邻域的大小
    ne.setInputCloud(cloud);               //输入的点云
    ne.compute(*normals);                    //计算法线

    //可视化
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");   //视口的名称
    viewer.setBackgroundColor (0.0, 0.0, 0.5);    //背景颜色的设置
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);  //将法线加入到点云中

     while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
}