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

void test_IntegralPCL7()
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

typedef pcl::PointXYZ PointType;

//参数的设置
float angular_resolution_7 = 0.5f;
float support_size_7 = 0.2f;
pcl::RangeImage::CoordinateFrame coordinate_frame_7 = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange_7 = false;
bool rotation_invariant_7 = true;

void printUsage_7(const char *progName)
{
    std::cout << "\n\nUsage: "<<progName<<" [options] <scene.pcd>\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-r <float>   angular resolution in degrees (default "<<angular_resolution_7<<")\n"
            << "-c <int>     coordinate frame (default "<< (int)coordinate_frame_7<<")\n"
            << "-m           Treat all unseen points to max range\n"
            << "-s <float>   support size for the interest points (diameter of the used sphere - "
                                                                  "default "<<support_size_7<<")\n"
            << "-o <0/1>     switch rotational invariant version of the feature on/off"
            <<               " (default "<< (int)rotation_invariant_7<<")\n"
            << "-h           this help\n"
            << "\n\n";
}

void setViewerPose_7(pcl::visualization::PCLVisualizer &viewer, const Eigen::Affine3f &viewer_pose)
{
    Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f (0, 0, 0);
    Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f (0, 0, 1) + pos_vector;
    Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f (0, -1, 0);
    viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);

}

int test_NarfPCL7(int argc, char **argv)
{
    // 设置参数检测
    if (pcl::console::find_argument (argc, argv, "-h") >= 0)
    {
        printUsage_7 (argv[0]);
        return 0;
    }
    if (pcl::console::find_argument (argc, argv, "-m") >= 0)
    {
        setUnseenToMaxRange_7 = true;
        cout << "Setting unseen values in range image to maximum range readings.\n";
    }
    if (pcl::console::parse (argc, argv, "-o", rotation_invariant_7) >= 0)
        cout << "Switching rotation invariant feature version "<< (rotation_invariant_7 ? "on" : "off")<<".\n";
    int tmp_coordinate_frame;
    if (pcl::console::parse (argc, argv, "-c", tmp_coordinate_frame) >= 0)
    {
        coordinate_frame_7 = pcl::RangeImage::CoordinateFrame (tmp_coordinate_frame);
        cout << "Using coordinate frame "<< (int)coordinate_frame_7<<".\n";
    }
    if (pcl::console::parse (argc, argv, "-s", support_size_7) >= 0)
        cout << "Setting support size to "<<support_size_7<<".\n";
    if (pcl::console::parse (argc, argv, "-r", angular_resolution_7) >= 0)
        cout << "Setting angular resolution to "<<angular_resolution_7<<"deg.\n";
    angular_resolution_7 = pcl::deg2rad (angular_resolution_7);

    //打开一个磁盘中的.pcd文件  但是如果没有指定就会自动生成
    pcl::PointCloud<PointType>::Ptr    point_cloud_ptr (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;

    pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
    Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
     
    std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
    if (!pcd_filename_indices.empty ())   //检测是否有far_ranges.pcd
    {
        std::string filename = argv[pcd_filename_indices[0]];
        if (pcl::io::loadPCDFile (filename, point_cloud) == -1)
        {
            cerr << "Was not able to open file \""<<filename<<"\".\n";
            printUsage_7 (argv[0]);
            return 0;
        }
        scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                               point_cloud.sensor_origin_[1],
                                                               point_cloud.sensor_origin_[2])) *
                                                               Eigen::Affine3f (point_cloud.sensor_orientation_);
        std::string far_ranges_filename = pcl::getFilenameWithoutExtension (filename)+"_far_ranges.pcd";
        if (pcl::io::loadPCDFile (far_ranges_filename.c_str (), far_ranges) == -1)
            std::cout << "Far ranges file \""<<far_ranges_filename<<"\" does not exists.\n";
    }
     else
    {
        setUnseenToMaxRange_7 = true;
        cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
        for (float x=-0.5f; x<=0.5f; x+=0.01f)   //如果没有打开的文件就生成一个矩形的点云
        {
            for (float y=-0.5f; y<=0.5f; y+=0.01f)
            {
                PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
                point_cloud.points.push_back (point);
        }
    }
    point_cloud.width = (int) point_cloud.points.size ();  
    point_cloud.height = 1;
    }

    //从点云中建立生成深度图
    float noise_level = 0.0;    
    float min_range = 0.0f;
    int border_size = 1;

    boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
    pcl::RangeImage& range_image = *range_image_ptr;   
    range_image.createFromPointCloud (point_cloud, angular_resolution_7, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                   scene_sensor_pose, coordinate_frame_7, noise_level, min_range, border_size);
    range_image.integrateFarRanges (far_ranges);

    if (setUnseenToMaxRange_7)
        range_image.setUnseenToMaxRange ();
    
     //打开3D viewer并加入点云
    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
    viewer.setBackgroundColor (1, 1, 1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
    
    viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
 
    viewer.initCameraParameters ();
    setViewerPose_7(viewer, range_image.getTransformationToWorldSystem ());

    //显示
    pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
    range_image_widget.showRangeImage (range_image);

    //提取NARF特征
    pcl::RangeImageBorderExtractor range_image_border_extractor;    //申明深度图边缘提取器
    pcl::NarfKeypoint narf_keypoint_detector;                       //narf_keypoint_detector为点云对象

    narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor);
    narf_keypoint_detector.setRangeImage (&range_image);

    narf_keypoint_detector.getParameters ().support_size = support_size_7;    //获得特征提取的大小
    pcl::PointCloud<int> keypoint_indices;
    narf_keypoint_detector.compute (keypoint_indices);
    std::cout << "Found "<<keypoint_indices.points.size ()<<" key points.\n";

    // ----------------------------------------------
    // -----Show keypoints in range image widget-----
    // ----------------------------------------------
    //for (size_t i=0; i<keypoint_indices.points.size (); ++i)
    //range_image_widget.markPoint (keypoint_indices.points[i]%range_image.width,
    //keypoint_indices.points[i]/range_image.width);

    //在3Dviewer显示提取的特征信息
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;

    keypoints.points.resize (keypoint_indices.points.size ());
    for (size_t i=0; i<keypoint_indices.points.size (); ++i)
    {
        keypoints.points[i].getVector3fMap () = range_image.points[keypoint_indices.points[i]].getVector3fMap ();
    }
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (keypoints_ptr, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ> (keypoints_ptr, keypoints_color_handler, "keypoints");

    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

    //在关键点提取NARF描述子
    std::vector<int> keypoint_indices2;
    keypoint_indices2.resize (keypoint_indices.points.size ());
    for (unsigned int i=0; i<keypoint_indices.size (); ++i) // This step is necessary to get the right vector type
    keypoint_indices2[i]=keypoint_indices.points[i];      ///建立NARF关键点的索引向量，此矢量作为NARF特征计算的输入来使用

    pcl::NarfDescriptor narf_descriptor (&range_image, &keypoint_indices2);//创建narf_descriptor对象。并给了此对象输入数据（特征点索引和深度像）
    narf_descriptor.getParameters ().support_size = support_size_7;//support_size确定计算描述子时考虑的区域大小
    narf_descriptor.getParameters ().rotation_invariant = rotation_invariant_7;    //设置旋转不变的NARF描述子
    pcl::PointCloud<pcl::Narf36> narf_descriptors;               //创建Narf36的点类型输入点云对象并进行实际计算
    narf_descriptor.compute (narf_descriptors);                 //计算描述子
    cout << "Extracted "<<narf_descriptors.size ()<<" descriptors for "   //打印输出特征点的数目和提取描述子的数目
                      <<keypoint_indices.points.size ()<< " keypoints.\n";

    //主循环函数
    while (!viewer.wasStopped ())
    {
        range_image_widget.spinOnce ();  // process GUI events
        viewer.spinOnce ();
        pcl_sleep(0.01);
    }

}