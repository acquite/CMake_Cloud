#include "T_PCL.h"

int user_data;

//回调-初始化执行一次
void viewerOneOff(pcl::visualization::PCLVisualizer &viewer)
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
void viewerPsycho(pcl::visualization::PCLVisualizer &viewer)
{
  static unsigned count = 0;
  std::stringstream ss;
  ss << "Once per viewer loop:" << count++;
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
  cloud.width = 5;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);

  for (auto &point : cloud)
  {
    point.x = 1024 * rand() / (RAND_MAX + 1.0f);
    point.y = 1024 * rand() / (RAND_MAX + 1.0f);
    point.z = 1024 * rand() / (RAND_MAX + 1.0f);
  }
  pcl::io::savePCDFileASCII("./data/test2.pcd", cloud);
  std::cerr << "Saved " << cloud.size() << "data points to test2.pcd." << std::endl;

  for (const auto &point : cloud)
  {
    std::cerr << "    " << point.x << "   " << point.y << "   " << point.z << std::endl;
  }
}

void showHelp(char *program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}

//https://pcl.readthedocs.io/projects/tutorials/en/latest/matrix_transform.html#matrix-transform
void test3(int argc, char **argv)
{
  // Show help
  if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
  {
    showHelp(argv[0]);
    return;
  }
  // Fetch point cloud filename in arguments | Works with PCD and PLY files
  std::vector<int> filenames;
  bool file_is_pcd = false;

  filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");
  if (filenames.size() != 1)
  {
    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
    if (filenames.size() != 1)
    {
      showHelp(argv[0]);
      return;
    }
    else
    {
      file_is_pcd = true;
    }
  }
  //Load file | works with PCD and PLY files
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  if (file_is_pcd)
  {
    if (pcl::io::loadPCDFile(argv[filenames[0]], *source_cloud) < 0)
    {
      std::cout << "Error loading point cloud" << argv[filenames[0]] << std::endl
                << std::endl;
      showHelp(argv[0]);
      return;
    }
  }
  else
  {
    if (pcl::io::loadPLYFile(argv[filenames[0]], *source_cloud) < 0)
    {
      std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl
                << std::endl;
      showHelp(argv[0]);
      return;
    }
  }

  /* Reminder: how transformation matrices work :

           |-------> This column is the translation
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

    METHOD #1: Using a Matrix4f
    This is the "manual" method, perfect to understand but error prone !
  */
  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
  // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
  float theta = M_PI / 4; // The angle of rotation in radians
  transform_1(0, 0) = std::cos(theta);
  transform_1(0, 1) = -sin(theta);
  transform_1(1, 0) = sin(theta);
  transform_1(1, 1) = cos(theta);
  //(row,column)

  // Define a translation of 2.5 meters on the x axis.
  transform_1(0, 3) = 2.5;
  // Print the transformation
  printf("Method #1: using a Matrix4f\n");
  std::cout << transform_1 << std::endl;

  /*  METHOD #2: Using a Affine3f
    This method is easier and less error prone
  */
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

  //Define a translation of 2.5 meters on the x axis.
  transform_2.translation() << 2.5, 0.0, 0.0;

  //The same rotation matrix as before; theta radians around Z axis
  transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

  //print the transformation
  printf("\nMethod #2:using an Affine3f\n");
  std::cout << transform_2.matrix() << std::endl;

  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2);

  //Visualization
  printf("\n Point cloud colors:white=original point cloud\n"
         "red=transformed point cloud\n");
  pcl::visualization::PCLVisualizer viewer("Matrix transformation example");

  //Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 255, 255, 255);

  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud(source_cloud, source_cloud_color_handler, "original_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformed_cloud, 230, 20, 20); // Red

  viewer.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

  viewer.addCoordinateSystem(1.0, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey

  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "transformed_cloud");

  while (!viewer.wasStopped())
  { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce();
  }
}

void test_passThrough()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  //填充点云数据
  cloud->width = 5;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud beforn filltering:" << std::endl;
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    std::cerr << "  " << cloud->points[i].x << "  "
              << cloud->points[i].y << "  "
              << cloud->points[i].z << std::endl;
  }

  //创建筛选对象
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  //保留【0,1】的点
  pass.setFilterLimits(0.0, 1.0);

  //保留【0,1】之外的点
  pass.setFilterLimitsNegative(true);
  pass.filter(*cloud_filtered);

  std::cerr << "Cloud after filtering:" << std::endl;
  for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
  {
    std::cerr << "  " << cloud_filtered->points[i].x << " "
              << cloud_filtered->points[i].y << "  "
              << cloud_filtered->points[i].z << std::endl;
  }
}

void test_Integral()
{
  //加载点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile("./data/table_scene_mug_stereo_textured.pcd", *cloud);

  //估计法线
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

  pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor(0.002f);
  ne.setNormalSmoothingSize(10.0f);
  ne.setInputCloud(cloud);
  ne.compute(*normals);

  //可视化法线
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor(0.0, 0.0, 0.5);
  viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals);

  while (!viewer.wasStopped())
  {
    viewer.spinOnce();
  }
}

typedef pcl::PointXYZ PointType;
//参数
float angular_resolution = 0.5f;
float support_size = 0.2f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;
bool rotation_invariant = true;

void printUsage(const char *progName);

void printUsage(const char *progName)
{
  std::cout << "\n\n Usage: " << progName << " [options]<scene.pcd>\n\n"
            << "Options:\n"
            << "------------------------------"
            << "-r <float> angular resolution in degrees(default " << angular_resolution << ")\n"
            << "-c <int> coordinate frame (default " << (int)coordinate_frame << ")\n"
            << "-m Treat all unseen points to max range\n"
            << "-s <float> support size for the interest points(diameter of the used sphere - "
               "default "
            << support_size << ")\n"
            << "-o <0/1> switch rotational invariant version of the feature on/off"
            << "(default" << (int)rotation_invariant << ")\n"
            << "-h this help\n"
            << "\n\n";
}

void setViewerPose(pcl::visualization::PCLVisualizer &viewer, const Eigen::Affine3f &viewer_pose)
{
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
  viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],
                           look_at_vector[0], look_at_vector[1], look_at_vector[2],
                           up_vector[0], up_vector[0], up_vector[2]);
}

void test_Narf(int argc, char **argv)
{
  //解析命令行参数
  if (pcl::console::find_argument(argc, argv, "-h") >= 0)
  {
    printUsage(argv[0]);
    return;
  }

  if (pcl::console::find_argument(argc, argv, "-m") >= 0)
  {
    setUnseenToMaxRange = true;
    std::cout << "Setting unseen values in range image to maximum range readings.\n";
  }

  if (pcl::console::parse(argc, argv, "-o", rotation_invariant) >= 0)
  {
    std::cout << "Switching rotation invariant feature version " << (rotation_invariant ? "on" : "off") << ".\n";
  }

  int tmp_coordinate_frame;
  if (pcl::console::parse(argc, argv, "-c", tmp_coordinate_frame) >= 0)
  {
    coordinate_frame = pcl::RangeImage::CoordinateFrame(tmp_coordinate_frame);
    std::cout << "Using coordinate frame" << (int)coordinate_frame << ".\n";
  }

  if (pcl::console::parse(argc, argv, "-s", support_size) > 0)
  {
    std::cout << "Setting support size to " << support_size << ".\n";
  }

  if (pcl::console::parse(argc, argv, "-r", angular_resolution) >= 0)
  {
    std::cout << "Setting angular resolution to" << angular_resolution << "deg.\n";
  }

  angular_resolution = pcl::deg2rad(angular_resolution);

  //读取pcd文件或创建示例点云(如果没有给出)
  pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType> &point_cloud = *point_cloud_ptr;
  pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
  Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
  std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument(argc, argv, "pcd");

  if (!pcd_filename_indices.empty())
  {
    std::string filename = argv[pcd_filename_indices[0]];
    if (pcl::io::loadPCDFile(filename, point_cloud) == -1)
    {
      std::cerr << "was not able to open file \"" << filename << "\".\n";
      printUsage(argv[0]);
      return;
    }

    scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0],
                                                             point_cloud.sensor_origin_[1],
                                                             point_cloud.sensor_origin_[2])) *
                        Eigen::Affine3f(point_cloud.sensor_orientation_);

    std::string far_ranges_filename = pcl::getFilenameWithoutExtension(filename) + "_far_ranges.pcd";
    if (pcl::io::loadPCDFile(far_ranges_filename.c_str(), far_ranges) == -1)
    {
      std::cout << "Far ranges file \"" << far_ranges_filename << "\" does not exists.\n";
    }
  }
  else
  {
    setUnseenToMaxRange = true;
    std::cout << "\n No *.pcd file given=>Generating example point cloud.\n\n";
    for (float x = -0.5f; x <= 0.5f; x += 0.01f)
    {
      for (float y = -0.5f; y <= 0.5f; y += 0.01)
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

  //从点云创建深度图像
  float noise_level = 0.0;
  float min_range = 0.0f;
  int border_size = 1;
  pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
  pcl::RangeImage &range_image = *range_image_ptr;
  range_image.createFromPointCloud(point_cloud, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f), scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);

  range_image.integrateFarRanges(far_ranges);
  if (setUnseenToMaxRange)
  {
    range_image.setUnseenToMaxRange();
  }

  // --------------------------------------------
  // -----打开3D查看器并添加点云-----
  // --------------------------------------------

  pcl::visualization::PCLVisualizer viewer("sD Viewer");
  viewer.setBackgroundColor(1, 1, 1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr, 0, 0, 0);

  viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");

  viewer.initCameraParameters();
  setViewerPose(viewer, range_image.getTransformationToWorldSystem());

  //显示深度图像
  pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
  range_image_widget.showRangeImage(range_image);

  //提取NARF关键点
  pcl::RangeImageBorderExtractor range_image_border_extractor;
  pcl::NarfKeypoint narf_keypoint_detector;
  narf_keypoint_detector.setRangeImageBorderExtractor(&range_image_border_extractor);
  narf_keypoint_detector.setRangeImage(&range_image);
  narf_keypoint_detector.getParameters().support_size = support_size;

  pcl::PointCloud<int> keypoint_indices;
  narf_keypoint_detector.compute(keypoint_indices);
  std::cout << "Found " << keypoint_indices.points.size() << " key points.\n";

  //显示深度图像小部件中的关键点

  //在3D查看器中显示关键点
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ> &keypoints = *keypoints_ptr;
  keypoints.points.resize(keypoint_indices.points.size());
  for (size_t i = 0; i < keypoint_indices.points.size(); ++i)
  {
    keypoints.points[i].getArray3fMap() = range_image.points[keypoint_indices.points[i]].getArray3fMap();
  }
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(keypoints_ptr, 0, 255, 0);
  viewer.addPointCloud<pcl::PointXYZ>(keypoints_ptr, keypoints_color_handler, "keypoints");

  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

  //提取感兴趣点的NARF描述符

  std::vector<int> keypoint_indices2;
  keypoint_indices2.resize(keypoint_indices.points.size());
  for (unsigned int i = 0; i < keypoint_indices.size(); ++i)
  {
    keypoint_indices2[i] = keypoint_indices.points[i];
  }
  pcl::NarfDescriptor narf_descriptor(&range_image, &keypoint_indices2);

  narf_descriptor.getParameters().support_size = support_size;
  narf_descriptor.getParameters().rotation_invariant = rotation_invariant;

  pcl::PointCloud<pcl::Narf36> narf_descriptors;
  narf_descriptor.compute(narf_descriptors);
  std::cout << "Extracted" << narf_descriptors.size() << "descriptors for" << keypoint_indices.points.size() << " keypoints.\n";

  //主循环
  while (!viewer.wasStopped())
  {
    range_image_widget.spinOnce();
    viewer.spinOnce();
    pcl_sleep(0.01);
  }
}

//视图特征直方图
void test_VFH()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

  pcl::VFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::VFHSignature308> vfh;
  vfh.setInputCloud(cloud);
  vfh.setInputNormals(normals);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  vfh.setSearchMethod(tree);

  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308>());

  vfh.compute(*vfhs);

}

void test_momentEst(int argc, char** argv)
{
  if (argc != 2)
    return ;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

  if(pcl::io::loadPCDFile(argv[1],*cloud)==-1)
    return;

  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(cloud);
  feature_extractor.compute ();

  std::vector <float> moment_of_inertia;
  std::vector <float> eccentricity;

  pcl::PointXYZ min_point_AABB;
  pcl::PointXYZ max_point_AABB;
  pcl::PointXYZ min_point_OBB;
  pcl::PointXYZ max_point_OBB;
  pcl::PointXYZ position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;
  float major_value, middle_value, minor_value;
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  Eigen::Vector3f mass_center;

  feature_extractor.getMomentOfInertia (moment_of_inertia);
  feature_extractor.getEccentricity (eccentricity);
  feature_extractor.getAABB (min_point_AABB, max_point_AABB);
  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
  feature_extractor.getEigenValues (major_value, middle_value, minor_value);
  feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
  feature_extractor.getMassCenter (mass_center);

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");

  Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
  Eigen::Quaternionf quat (rotational_matrix_OBB);
  viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

  pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
  pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
  pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
  pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
  viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
  viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
  viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

  while(!viewer->wasStopped())
  {
    viewer->spinOnce (100);
    // std::this_thread::sleep_for(100);
    pcl_sleep(0.01);
  }
}


void test_rops(int argc, char** argv)
{
  if (argc != 4)
    return;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  if (pcl::io::loadPCDFile (argv[1], *cloud) == -1)
    return ;
  
  pcl::PointIndicesPtr indices(new pcl::PointIndices);
  std::ifstream indices_file;
  indices_file.open(argv[2], std::ifstream::in);
  for (std::string line; std::getline (indices_file, line);)
  {
    std::istringstream in(line);
    unsigned int index = 0;
    in>>index;
    indices->indices.push_back (index - 1);
  }
  indices_file.close ();

  std::vector <pcl::Vertices> triangles;
  std::ifstream triangles_file;
  triangles_file.open (argv[3], std::ifstream::in);
  for (std::string line; std::getline (triangles_file, line);)
  {
    pcl::Vertices triangle;
    std::istringstream in (line);
    unsigned int vertex = 0;
    in >> vertex;
    triangle.vertices.push_back (vertex - 1);
    in >> vertex;
    triangle.vertices.push_back (vertex - 1);
    triangles.push_back (triangle);
  }

  float support_radius = 0.0285f;
  unsigned int number_of_partition_bins = 5;
  unsigned int number_of_rotations = 3;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr search_method (new pcl::search::KdTree<pcl::PointXYZ>);
  search_method->setInputCloud (cloud);

  pcl::ROPSEstimation <pcl::PointXYZ, pcl::Histogram <135> > feature_estimator;
  feature_estimator.setSearchMethod (search_method);
  feature_estimator.setSearchSurface (cloud);
  feature_estimator.setInputCloud (cloud);
  feature_estimator.setIndices (indices);
  feature_estimator.setTriangles (triangles);
  feature_estimator.setRadiusSearch (support_radius);
  feature_estimator.setNumberOfPartitionBins (number_of_partition_bins);
  feature_estimator.setNumberOfRotations (number_of_rotations);
  feature_estimator.setSupportRadius (support_radius);

  pcl::PointCloud<pcl::Histogram <135> >::Ptr histograms (new pcl::PointCloud <pcl::Histogram <135> > ());
  feature_estimator.compute (*histograms);
}

void test_gasd()
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
   // 创建GASD估计类，并将输入数据集传递给它

}

void test_passThrough2()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  //填充点云数据
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud before filtering: " << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
  std::cerr << "    " << cloud->points[i].x << " " 
                        << cloud->points[i].y << " " 
                        << cloud->points[i].z << std::endl;

  // 创建筛选对象
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
  std::cerr << "    " << cloud_filtered->points[i].x << " " 
                        << cloud_filtered->points[i].y << " " 
                        << cloud_filtered->points[i].z << std::endl;
}

void test_voxelGrid()
{
  pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
  pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

  //填充点云数据
  pcl::PCDReader reader;
  // 将下面的路径替换为您保存文件的路径
  reader.read("../data/table_scene_lms400.pcd",*cloud);
  std::cerr<<"PointCloud before filtering:"<<cloud->width*cloud->height<<"data points("<<pcl::getFieldsList(*cloud)<<").";

  //创建筛选对象
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloud);
  // 叶子大小为1cm
  sor.setLeafSize(0.01f,0.01f,0.01f);
  sor.filter(*cloud_filtered);
  std::cerr<<"PointCloud after filtering:"<<cloud_filtered->width*cloud_filtered->height<<"data points("<<pcl::getFieldsList(*cloud_filtered)<<").";

  pcl::PCDWriter writer;
  writer.write("table_scene_lms400_downsampled.pcd",*cloud_filtered,Eigen::Vector4f::Zero(),Eigen::Quaternionf::Identity(),false);

}

void test_statistical()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);


   //填充点云数据
  pcl::PCDReader reader;
  //将下面的路径替换为您保存文件的路径
  reader.read<pcl::PointXYZ> ("../data/table_scene_lms400.pcd", *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;
  // 创建筛选对象
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  //每个点要分析的邻居数量设置为50，标准偏差乘数为1。这意味着，所有距离查询点的平均距离大于1个标准差的点都将被标记为离群点并删除
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

  //获得离群值
  sor.setNegative(true);
  sor.filter(*cloud_filtered);
  writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);
}

void test_Model()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

   //填充点云数据
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud before projection: " << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " " 
                        << cloud->points[i].y << " " 
                        << cloud->points[i].z << std::endl;

  // 用X=Y=0,Z=1创建一组平面系数
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;

  //创建筛选对象
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);

  std::cerr << "Cloud after projection: " << std::endl;
  for (size_t i = 0; i < cloud_projected->points.size (); ++i)
    std::cerr << "    " << cloud_projected->points[i].x << " " 
                        << cloud_projected->points[i].y << " " 
                        << cloud_projected->points[i].z << std::endl;

}