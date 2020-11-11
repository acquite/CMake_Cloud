#include"T_PCLF1.h"

void test_InitPCLF1()
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = test_readPCD_PCLF1();

    test_showCloud_PCLF1(cloud);
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr test_readPCD_PCLF1()
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::PCDReader reader;
    reader.read<pcl::PointXYZRGBA>("../data/988.pcd",*cloud);

    std::cout << "PointCloud before filtering has:" << cloud->points.size() << "data points." << std::endl;

    return cloud;
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