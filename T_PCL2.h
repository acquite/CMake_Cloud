#include <iostream>
#include <pcl-1.8/pcl/io/pcd_io.h>           //PCD读写类相关的头文件
#include <pcl-1.8/pcl/point_types.h>      //PCL中支持的点类型的头文件
void test_readPCD();

void test_loadPCD();

// 连接两个点云中的字段或数据形成新点云
void test_addCloud(int argc,char **argv);

//给点云添加高斯噪声：给坐标添加随机数
#include <pcl/point_cloud.h>
#include <pcl/console/time.h>
#include <boost/random.hpp>
void test_gs();

//kd-tree 的实现
#include<pcl-1.8/pcl/kdtree/kdtree_flann.h>
#include<vector>
#include<ctime>
void test_kdtree();

//利用八叉树进行点云压缩
#include<pcl/io/openni2_grabber.h>//点云获取接口类
#include<pcl/visualization/cloud_viewer.h>//点云可视化类
#include<pcl/compression/octree_pointcloud_compression.h>//点云压缩类
#include<stdio.h>
#include<sstream>
#include<stdlib.h>

#ifdef WIN32
#define sleep(x) Sleep((x)*1000)
#endif

class SimpleOpenNIViewer
{
public:
    pcl::visualization::CloudViewer viewer;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudEncoder;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudDecoder;

    SimpleOpenNIViewer () :
    viewer (" Point Cloud Compression Example")
    {
    }
    /************************************************************************************************
  在OpenNIGrabber采集循环执行的回调函数cloud_cb_中，首先把获取的点云压缩到stringstream缓冲区，下一步就是解压缩，
  它对压缩了的二进制数据进行解码，存储在新的点云中解码了点云被发送到点云可视化对象中进行实时可视化
    *************************************************************************************************/
  
    void  cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {  
        if (!viewer.wasStopped ())
        {
            // 存储压缩点云的字节流对象
            std::stringstream compressedData;
            // 存储输出点云
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZRGBA> ());
            // 压缩点云
            PointCloudEncoder->encodePointCloud (cloud, compressedData);
            // 解压缩点云
            PointCloudDecoder->decodePointCloud (compressedData, cloudOut);
            // 可视化解压缩的点云
            viewer.showCloud (cloudOut);
        }
    }
    /**************************************************************************************************************
 在函数中创建PointCloudCompression类的对象来编码和解码，这些对象把压缩配置文件作为配置压缩算法的参数
 所提供的压缩配置文件为OpenNI兼容设备采集到的点云预先确定的通用参数集，本例中使用MED_RES_ONLINE_COMPRESSION_WITH_COLOR
 配置参数集，用于快速在线的压缩，压缩配置方法可以在文件/io/include/pcl/compression/compression_profiles.h中找到，
  在PointCloudCompression构造函数中使用MANUAL——CONFIGURATION属性就可以手动的配置压缩算法的全部参数
******************************************************************************************************************/

    void run()
    {
        bool showStatistics = true;  //设置在标准设备上输出打印出压缩结果信息
        // 压缩选项详情在: /io/include/pcl/compression/compression_profiles.h
        pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
        // 初始化压缩和解压缩对象  其中压缩对象需要设定压缩参数选项，解压缩按照数据源自行判断
        PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> (compressionProfile, showStatistics);
        PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> ();

    /***********************************************************************************************************
    下面的代码为OpenNI兼容设备实例化一个新的采样器，并且启动循环回调接口，每从设备获取一帧数据就回调函数一次，，这里的
    回调函数就是实现数据压缩和可视化解压缩结果。
   ************************************************************************************************************/

        //创建从OpenNI获取点云的抓取对象
        pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();
        // 建立回调函数
         boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

        //建立回调函数和回调信息的绑定
        boost::signals2::connection c = interface->registerCallback (f);

        // 开始接受点云的数据流
        interface->start ();

        while (!viewer.wasStopped ())
        {
            sleep (1);
        }

        interface->stop ();
        // 删除压缩与解压缩的实例
        delete (PointCloudEncoder);
        delete (PointCloudDecoder);
    }

};

void test_coder();


//可视化（经典圆球测试）
/**********************************************************************************
  函数是作为回调函数，在主函数中只注册一次 ，函数实现对可视化对象背景颜色的设置，添加一个圆球几何体
*********************************************************************************/

void viewerOneOff (pcl::visualization::PCLVisualizer& viewer);

/***********************************************************************************
   作为回调函数，在主函数中注册后每帧显示都执行一次，函数具体实现在可视化对象中添加一个
*************************************************************************************/ 
void viewerPsycho (pcl::visualization::PCLVisualizer& viewer);

/**************************************************************
  首先加载点云文件到点云对象，并初始化可视化对象viewer，注册上面的回
   调函数，执行循环直到收到关闭viewer的消息退出程序
*************************************************************/

void test_shape();


//基于octree的空间划分及搜索操作
#include <pcl/octree/octree.h>
void test_octree();

// PCL点云类型的转换
#include<pcl-1.8/pcl/common/impl/io.hpp>
void test_change();



//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------【2】------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
// 最基本的点云可视化操作
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

// 可视化彩色点云颜色特征
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

//将点云赋值为指定颜色（自定义颜色）
boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

//可视化点云法线和其他特征
//显示法线是理解点云的一个重要步骤，点云法线特征是非常重要的基础特征，PCL visualizer可视化类可用于绘制法线，
//也可以绘制表征点云的其他特征，比如主曲率和几何特征，normalsVis函数中演示了如何实现点云的法线
boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis(
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals);