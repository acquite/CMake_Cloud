#include "T_PCL6.h"

void test_rImgPCL6()
{
    pcl::PointCloud<pcl::PointXYZ> pointCloud; //定义点云的对象
    // 循环产生点云的数据
    for (float y = -0.5f; y <= 0.5f; y += 0.01f)
    {
        for (float z = -0.5f; z <= 0.5f; z += 0.01f)
        {
            pcl::PointXYZ point;
            point.x = 2.0f - y;
            point.y = y;
            point.z = z;
            pointCloud.points.push_back(point); //循环添加点数据到点云对象
        }
    }
    pointCloud.width = (uint32_t)pointCloud.points.size();
    pointCloud.height = 1; //设置点云对象的头信息
    //实现一个呈矩形形状的点云
    // We now want to create a range image from the above point cloud, with a 1deg angular resolution
    //angular_resolution为模拟的深度传感器的角度分辨率，即深度图像中一个像素对应的角度大小
    float angularResolution = (float)(1.0f * (M_PI / 180.0f)); //   1.0 degree in radians
    // /max_angle_width为模拟的深度传感器的水平最大采样角度，
    float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f)); // 360.0 degree in radians
    //max_angle_height为模拟传感器的垂直方向最大采样角度  都转为弧度
    float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f)); // 180.0 degree in radians
    //传感器的采集位置
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    //深度图像遵循坐标系统
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel = 0.00; //noise_level获取深度图像深度时，近邻点对查询点距离值的影响水平
    float minRange = 0.0f;   //min_range设置最小的获取距离，小于最小获取距离的位置为传感器的盲区
    int borderSize = 1;      //border_size获得深度图像的边缘的宽度

    pcl::RangeImage rangeImage;
    rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

    std::cout << rangeImage << "\n";
}

typedef pcl::PointXYZ PointType;
//参数
float angular_resolution_x6 = 0.5f,
      angular_resolution_y6 = angular_resolution_x6; //angular_resolution为模拟的深度传感器的角度分辨率，即深度图像中一个像素对应的角度大小

pcl::RangeImage::CoordinateFrame coordinate_frame_o6 = pcl::RangeImage::CAMERA_FRAME; //深度图像遵循坐标系统
bool live_update_6 = false;

//命令帮助提示
void printUsage_6(const char *progName)
{
    std::cout << "\n\nUsage: " << progName << " [options] <scene.pcd>\n\n"
              << "Options:\n"
              << "-------------------------------------------\n"
              << "-rx <float>  angular resolution in degrees (default " << angular_resolution_x6 << ")\n"
              << "-ry <float>  angular resolution in degrees (default " << angular_resolution_y6 << ")\n"
              << "-c <int>     coordinate frame (default " << (int)coordinate_frame_o6 << ")\n"
              << "-l           live update - update the range image according to the selected view in the 3D viewer.\n"
              << "-h           this help\n"
              << "\n\n";
}

void setViewerPose_6(pcl::visualization::PCLVisualizer &viewer, const Eigen::Affine3f &viewer_pose)
{
    Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
    Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
    viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],
                             look_at_vector[0], look_at_vector[1], look_at_vector[2],
                             up_vector[0], up_vector[1], up_vector[2]);
}

int test_ordarRImgPCL6(int argc, char **argv)
{
    //输入命令分析
    if (pcl::console::find_argument(argc, argv, "-h") >= 0)
    {
        printUsage_6(argv[0]);
        return 0;
    }
    if (pcl::console::find_argument(argc, argv, "-l") >= 0)
    {
        live_update_6 = true;
        std::cout << "Live update is on.\n";
    }
    if (pcl::console::parse(argc, argv, "-rx", angular_resolution_x6) >= 0)
        std::cout << "Setting angular resolution in x-direction to " << angular_resolution_x6 << "deg.\n";
    if (pcl::console::parse(argc, argv, "-ry", angular_resolution_y6) >= 0)
        std::cout << "Setting angular resolution in y-direction to " << angular_resolution_y6 << "deg.\n";

    int tmp_coordinate_frame;
    if (pcl::console::parse(argc, argv, "-c", tmp_coordinate_frame) >= 0)
    {
        coordinate_frame_o6 = pcl::RangeImage::CoordinateFrame(tmp_coordinate_frame);
        std::cout << "Using coordinate frame " << (int)coordinate_frame_o6 << ".\n";
    }
    angular_resolution_x6 = pcl::deg2rad(angular_resolution_x6);
    angular_resolution_y6 = pcl::deg2rad(angular_resolution_y6);

    //读取点云PCD文件  如果没有输入PCD文件就生成一个点云
    pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType> &point_cloud = *point_cloud_ptr;
    Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity()); //申明传感器的位置是一个4*4的仿射变换
    std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument(argc, argv, "pcd");
    if (!pcd_filename_indices.empty())
    {
        std::string filename = argv[pcd_filename_indices[0]];
        if (pcl::io::loadPCDFile(filename, point_cloud) == -1)
        {
            std::cout << "Was not able to open file \"" << filename << "\".\n";
            printUsage_6(argv[0]);
            return 0;
        }
        //给传感器的位姿赋值  就是获取点云的传感器的的平移与旋转的向量
        scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0], point_cloud.sensor_origin_[1], point_cloud.sensor_origin_[2])) * Eigen::Affine3f(point_cloud.sensor_orientation_);
    }
    else
    {
        //如果没有给点云，则我们要自己生成点云
        std::cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
        for (float x = -0.5f; x <= 0.5f; x += 0.01f)
        {
            for (float y = -0.5f; y <= 0.5f; y += 0.01f)
            {
                PointType point;
                point.x = x;
                point.y = y;
                point.z = 2.0f - y;
                point_cloud.points.push_back(point);
            }
        }
        point_cloud.width = (int)point_cloud.points.size();
        point_cloud.height = 1;
    }

    // -----从创建的点云中获取深度图--//
    //设置基本参数
    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;
    boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);

    pcl::RangeImage &range_image = *range_image_ptr;
    /*
 关于range_image.createFromPointCloud（）参数的解释 （涉及的角度都为弧度为单位） ：
   point_cloud为创建深度图像所需要的点云
  angular_resolution_x深度传感器X方向的角度分辨率
  angular_resolution_y深度传感器Y方向的角度分辨率
   pcl::deg2rad (360.0f)深度传感器的水平最大采样角度
   pcl::deg2rad (180.0f)垂直最大采样角度
   scene_sensor_pose设置的模拟传感器的位姿是一个仿射变换矩阵，默认为4*4的单位矩阵变换
   coordinate_frame定义按照那种坐标系统的习惯  默认为CAMERA_FRAME
   noise_level  获取深度图像深度时，邻近点对查询点距离值的影响水平
   min_range 设置最小的获取距离，小于最小的获取距离的位置为传感器的盲区
   border_size  设置获取深度图像边缘的宽度 默认为0 
*/
    range_image.createFromPointCloud(point_cloud, angular_resolution_x6, angular_resolution_y6, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f), scene_sensor_pose, coordinate_frame_o6, noise_level, min_range, border_size);

    //可视化点云
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(1, 1, 1);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr, 0, 0, 0);

    viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");

    //viewer.addCoordinateSystem (1.0f, "global");
    //PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
    //viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");

    viewer.initCameraParameters();
    //range_image.getTransformationToWorldSystem ()的作用是获取从深度图像坐标系统（应该就是传感器的坐标）转换为世界坐标系统的转换矩阵
    setViewerPose_6(viewer, range_image.getTransformationToWorldSystem()); //设置视点的位置

    //可视化深度图
    pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
    range_image_widget.showRangeImage(range_image);

    while (!viewer.wasStopped())
    {
        range_image_widget.spinOnce();
        viewer.spinOnce();
        pcl_sleep(0.01);

        if (live_update_6)
        {
            //如果选择的是——l的参数说明就是要根据自己选择的视点来创建深度图。
            // live update - update the range image according to the selected view in the 3D viewer.

            scene_sensor_pose = viewer.getViewerPose();

            range_image.createFromPointCloud(point_cloud, angular_resolution_x6, angular_resolution_y6, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f), scene_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range, border_size);

            range_image_widget.showRangeImage(range_image);
        }
    }
}

// --------------------
// -----Parameters-----
// --------------------
float angular_resolution_6 = 0.5f;
pcl::RangeImage::CoordinateFrame coordinate_frame_b6 = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange_6 = false;

void printUsage_b6(const char *progName)
{
    std::cout << "\n\nUsage: " << progName << " [options] <scene.pcd>\n\n"
              << "Options:\n"
              << "-------------------------------------------\n"
              << "-r <float>   angular resolution in degrees (default " << angular_resolution_6 << ")\n"
              << "-c <int>     coordinate frame (default " << (int)coordinate_frame_b6 << ")\n"
              << "-m           Treat all unseen points to max range\n"
              << "-h           this help\n"
              << "\n\n";
}

int test_borderPCL6(int argc, char **argv)
{
    // --------------------------------------
    // -----Parse Command Line Arguments-----
    // --------------------------------------
    if (pcl::console::find_argument(argc, argv, "-h") >= 0)
    {
        printUsage_b6(argv[0]);
        return 0;
    }
    if (pcl::console::find_argument(argc, argv, "-m") >= 0)
    {
        setUnseenToMaxRange_6 = true;
        cout << "Setting unseen values in range image to maximum range readings.\n";
    }
    int tmp_coordinate_frame;
    if (pcl::console::parse(argc, argv, "-c", tmp_coordinate_frame) >= 0)
    {
        coordinate_frame_b6 = pcl::RangeImage::CoordinateFrame(tmp_coordinate_frame);
        cout << "Using coordinate frame " << (int)coordinate_frame_b6 << ".\n";
    }
    if (pcl::console::parse(argc, argv, "-r", angular_resolution_6) >= 0)
    {
        cout << "Setting angular resolution to " << angular_resolution_6 << "deg.\n";
    }
    angular_resolution_6 = pcl::deg2rad(angular_resolution_6);
    // ------------------------------------------------------------------
    // -----Read pcd file or create example point cloud if not given-----
    // ------------------------------------------------------------------
    pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType> &point_cloud = *point_cloud_ptr;
    pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
    Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity()); //传感器的位置

    std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument(argc, argv, "pcd");

    if (!pcd_filename_indices.empty())
    {
        std::string filename = argv[pcd_filename_indices[0]];
        if (pcl::io::loadPCDFile(filename, point_cloud) == -1) //打开文件
        {
            cout << "Was not able to open file \"" << filename << "\".\n";
            printUsage_b6(argv[0]);
            return 0;
        }
        scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0], point_cloud.sensor_origin_[1], point_cloud.sensor_origin_[2])) * Eigen::Affine3f(point_cloud.sensor_orientation_); //仿射变换矩阵

        std::string far_ranges_filename = pcl::getFilenameWithoutExtension(filename) + "_far_ranges.pcd";
        if (pcl::io::loadPCDFile(far_ranges_filename.c_str(), far_ranges) == -1)
        {
            std::cout << "Far ranges file \"" << far_ranges_filename << "\" does not exists.\n";
        }
    }
    else
    {
        cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
        for (float x = -0.5f; x <= 0.5f; x += 0.01f) //填充一个矩形的点云
        {
            for (float y = -0.5f; y <= 0.5f; y += 0.01f)
            {
                PointType point;
                point.x = x;
                point.y = y;
                point.z = 2.0f - y;
                point_cloud.points.push_back(point);
            }
        }
        point_cloud.width = (int)point_cloud.points.size();
        point_cloud.height = 1;
    }

    // -----------------------------------------------
    // -----Create RangeImage from the PointCloud-----
    // -----------------------------------------------
    float noise_level = 0.0; //各种参数的设置
    float min_range = 0.0f;
    int border_size = 1;
    boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
    pcl::RangeImage &range_image = *range_image_ptr;

    range_image.createFromPointCloud(point_cloud, angular_resolution_6, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
                                     scene_sensor_pose, coordinate_frame_b6, noise_level, min_range, border_size);
    range_image.integrateFarRanges(far_ranges);

    if (setUnseenToMaxRange_6)
        range_image.setUnseenToMaxRange();
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer viewer("3D Viewer"); //创建视口
    viewer.setBackgroundColor(1, 1, 1);                    //设置背景颜色
    viewer.addCoordinateSystem(1.0f);                      //设置坐标系
    pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler(point_cloud_ptr, 0, 0, 0);
    viewer.addPointCloud(point_cloud_ptr, point_cloud_color_handler, "original point cloud"); //添加点云
    //PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 150, 150, 150);
    //viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
    //viewer.setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 2, "range image");

    // -------------------------
    // -----Extract borders提取边界的部分-----
    // -------------------------
    pcl::RangeImageBorderExtractor border_extractor(&range_image);
    pcl::PointCloud<pcl::BorderDescription> border_descriptions;
    border_extractor.compute(border_descriptions); //提取边界计算描述子

    // -------------------------------------------------------
    // -----Show points in 3D viewer在3D 视口中显示点云-----
    // ----------------------------------------------------
    pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>), //物体边界
        veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),                                         //veil边界
        shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);                                       //阴影边界

    pcl::PointCloud<pcl::PointWithRange> &border_points = *border_points_ptr,
                                         &veil_points = *veil_points_ptr,
                                         &shadow_points = *shadow_points_ptr;

    for (int y = 0; y < (int)range_image.height; ++y)
    {
        for (int x = 0; x < (int)range_image.width; ++x)
        {
            if (border_descriptions.points[y * range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
                border_points.points.push_back(range_image.points[y * range_image.width + x]);
            if (border_descriptions.points[y * range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
                veil_points.points.push_back(range_image.points[y * range_image.width + x]);
            if (border_descriptions.points[y * range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
                shadow_points.points.push_back(range_image.points[y * range_image.width + x]);
        }
    }
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> border_points_color_handler(border_points_ptr, 0, 255, 0);
    viewer.addPointCloud<pcl::PointWithRange>(border_points_ptr, border_points_color_handler, "border points");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> veil_points_color_handler(veil_points_ptr, 255, 0, 0);
    viewer.addPointCloud<pcl::PointWithRange>(veil_points_ptr, veil_points_color_handler, "veil points");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "veil points");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> shadow_points_color_handler(shadow_points_ptr, 0, 255, 255);
    viewer.addPointCloud<pcl::PointWithRange>(shadow_points_ptr, shadow_points_color_handler, "shadow points");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "shadow points");
    //-------------------------------------
    // -----Show points on range image-----
    // ------------------------------------
    pcl::visualization::RangeImageVisualizer *range_image_borders_widget = NULL;
    range_image_borders_widget =
        pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget(range_image, -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), false, border_descriptions, "Range image with borders");
    // -------------------------------------

    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer.wasStopped())
    {
        range_image_borders_widget->spinOnce();
        viewer.spinOnce();
        pcl_sleep(0.01);
    }
}
