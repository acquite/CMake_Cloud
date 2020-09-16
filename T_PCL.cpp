#include "T_PCL.h"

int user_data;

//回调-初始化执行一次
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    // viewer.setBackgroundColor (1.0, 0.5, 1.0);
    // pcl::PointXYZ o;
    // o.x = 1.0;
    // o.y = 0;
    // o.z = 0;
    // viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;
}

//回调-每帧显示
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count=0;
    std::stringstream ss;
    ss<<"Once per viewer loop:"<<count++;
    // viewer.removeShape("text",0);
    // viewer.addText(ss.str(),200,300,"text",0);
    user_data++;
}

void test1()
{
    // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    // pcl::io::loadPCDFile("data/maize.pcd",*cloud);//加载点云文件
    // pcl::visualization::CloudViewer viewer("Cloud Viewer");
    // viewer.showCloud(cloud);
    // viewer.runOnVisualizationThreadOnce(viewerOneOff);
    // viewer.runOnVisualizationThread(viewerPsycho);

    // while(!viewer.wasStopped())
    // {
    //     user_data++;
    // }


    // pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::io::loadPCDFile("data/maize.pcd", *Cloud);//读入点云数据
    // pcl::visualization::PCLVisualizer viewer;
    // viewer.setBackgroundColor(0, 0, 0);
    // pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(Cloud, "z");//按照z字段进行渲染
}

void test2()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
     // Fill in the cloud data
  cloud.width    = 5;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  for(auto& point: cloud)
  {
      point.x=1024*rand()/(RAND_MAX+1.0f);
      point.y=1024*rand()/(RAND_MAX+1.0f);
      point.z=1024*rand()/(RAND_MAX+1.0f);
  }
    pcl::io::savePCDFileASCII("./data/test2.pcd",cloud);
    std::cerr<<"Saved "<<cloud.size()<<"data points to test2.pcd."<<std::endl;

    for(const auto& point:cloud)
    {
        std::cerr<<"    "<<point.x<<"   "<<point.y<<"   "<<point.z<<std::endl;
    }
}