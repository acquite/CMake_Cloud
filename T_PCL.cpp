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