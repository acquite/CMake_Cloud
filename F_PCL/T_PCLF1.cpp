#include"T_PCLF1.h"


void test_readPCD_PCLF1()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>("../data/my_color.pcd",*cloud);

    std::cout << "PointCloud before filtering has:" << cloud->points.size() << "data points." << std::endl;

}

void test_showCloud_PCLF1(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
    //创建viewer对象
    pcl::visualization::CloudViewer viewer("Show Cloud");
    viewer.showCloud(cloud);

    while (!viewer.wasStopped ())
    {
        /* code */
    }
    

}