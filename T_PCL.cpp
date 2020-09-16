#include "T_PCL.h"

int user_data;

//回调-初始化执行一次
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(1.0,0.5,1.0);//设置背景颜色
    pcl::PointXYZ o;//存储球的圆心位置
    o.x=1.0;
    o.y=0;
    o.z=0;
    viewer.addSphere(o,0.25,"sphare",0);//添加圆球几何对象
    std::cout<<"Only run once."<<std::endl;
}

//回调-每帧显示
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count=0;
    std::stringstream ss;
    ss<<"Once per viewer loop:"<<count++;
    viewer.removeShape("text",0);
    viewer.addText(ss.str(),200,300,"text",0);
    user_data++;
}

void test1()
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    // pcl::io::loadPCDFile("")
}