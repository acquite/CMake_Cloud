#include"T_PCL8.h"

int test_mlsPCL8()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    pcl::io::loadPCDFile ("../data/my_color.pcd", *cloud);

    // 创建 KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;

    // 定义最小二乘实现的对象mls
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals (true);  //设置在最小二乘计算中需要进行法线估计

    // Set parameters
    mls.setInputCloud (cloud);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.03);

    // Reconstruct
    mls.process (mls_points);
    // Save output
    pcl::io::savePCDFile ("PCL8-mls.pcd", mls_points);

     //可视化
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer->setBackgroundColor (0.0, 0.0, 0.0);

    viewer->addPointCloud(cloud,"sample cloud");
    

    //视点默认坐标是（0，0，0）可使用 setViewPoint(float vpx,float vpy,float vpz)来更换
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }
}

int test_PlanePCL8()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), 
                                      cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), 
                                      cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::PCDReader reader;
    reader.read ("../data/table_scene_mug_stereo_textured.pcd", *cloud);
    // 建立过滤器消除杂散的NaN
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);                  //设置输入点云
    pass.setFilterFieldName ("z");             //设置分割字段为z坐标
    pass.setFilterLimits (0, 1.1);             //设置分割阀值为(0, 1.1)
    pass.filter (*cloud_filtered);   
    std::cerr << "PointCloud after filtering has: "
            << cloud_filtered->points.size () << " data points." << std::endl;
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);   //inliers存储分割后的点云
    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // 设置优化系数，该参数为可选参数
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    std::cerr << "PointCloud after segmentation has: "
            << inliers->indices.size () << " inliers." << std::endl;
    
    // Project the model inliers
    pcl::ProjectInliers<pcl::PointXYZ> proj;//点云投影滤波模型
    proj.setModelType (pcl::SACMODEL_PLANE); //设置投影模型
    proj.setIndices (inliers);
    proj.setInputCloud (cloud_filtered);
    proj.setModelCoefficients (coefficients);      //将估计得到的平面coefficients参数设置为投影平面模型系数
    proj.filter (*cloud_projected);            //得到投影后的点云
    std::cerr << "PointCloud after projection has: "
            << cloud_projected->points.size () << " data points." << std::endl;


    // 存储提取多边形上的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> chull;        //创建多边形提取对象
    chull.setInputCloud (cloud_projected);       //设置输入点云为提取后点云
    chull.setAlpha (0.1);
    chull.reconstruct (*cloud_hull);   //创建提取创建凹多边形

    std::cerr << "Concave hull has: " << cloud_hull->points.size ()
            << " data points." << std::endl;

    pcl::PCDWriter writer;
    writer.write ("../data/PCL8_table_scene_mug_stereo_textured_hull.pcd", *cloud_hull, false);

    return (0);
}

int test_TripPCL8()
{
    // 将一个XYZ点类型的PCD文件打开并存储到对象中
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile ("../data/my_color.pcd", cloud_blob);

    pcl::fromPCLPointCloud2 (cloud_blob, *cloud);

    //* the data should be available in cloud
    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;      //法线估计对象
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);   //存储估计的法线
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);  //定义kd树指针
    tree->setInputCloud (cloud);   ///用cloud构建tree对象

    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute(*normals);
    //   估计法线存储到其中
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);

    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);    //连接字段
    //* cloud_with_normals = cloud + normals

    //定义搜索树对象
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);

    tree2->setInputCloud (cloud_with_normals);   //点云构建搜索树

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   //定义三角化对象
    pcl::PolygonMesh triangles;                //存储最终三角化的网络模型

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.025);  //设置连接点之间的最大距离，（即是三角形最大边长）

    // 设置各参数值
    gp3.setMu (2.5);  //设置被样本点搜索其近邻点的最远距离为2.5，为了使用点云密度的变化


}

int test_ICPPCL8()
{
    //创建两个pcl::PointCloud<pcl::PointXYZ>共享指针，并初始化它们
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);


    // 随机填充点云
    cloud_in->width    = 5;               //设置点云宽度
    cloud_in->height   = 1;               //设置点云为无序点
    cloud_in->is_dense = false;
    cloud_in->points.resize (cloud_in->width * cloud_in->height);
    for (size_t i = 0; i < cloud_in->points.size (); ++i)
    {
        cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }
    std::cout << "Saved " << cloud_in->points.size () << " data points to input:"//打印处点云总数
      << std::endl;

    for (size_t i = 0; i < cloud_in->points.size (); ++i) 
        std::cout << "    " <<    //打印处实际坐标
        cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
        cloud_in->points[i].z << std::endl;
    *cloud_out = *cloud_in;
    std::cout << "size:" << cloud_out->points.size() << std::endl;

    //实现一个简单的点云刚体变换，以构造目标点云
    for (size_t i = 0; i < cloud_in->points.size (); ++i)
    {
        cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
    }
    std::cout << "Transformed " << cloud_in->points.size () << " data points:"
      << std::endl;
    
    for (size_t i = 0; i < cloud_out->points.size (); ++i)     //打印构造出来的目标点云
        std::cout << "    " << cloud_out->points[i].x << " " <<
        cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; 
    //创建IterativeClosestPoint的对象
    icp.setInputCloud(cloud_in);
    //cloud_in设置为点云的源点
    icp.setInputTarget(cloud_out);               //cloud_out设置为与cloud_in对应的匹配目标

    pcl::PointCloud<pcl::PointXYZ> Final;         //存储经过配准变换点云后的点云
    icp.align(Final);                             //打印配准相关输入信息

    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;

    std::cout << icp.getFinalTransformation() << std::endl;
    return 0;
}

//定义
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;   //申明pcl::PointXYZ数据
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// 申明一个全局可视化对象变量，定义左右视点分别显示配准前和配准后的结果点云
pcl::visualization::PCLVisualizer *p;     //创建可视化对象 
int vp_1, vp_2;                           //定义存储 左 右视点的ID

//申明一个结构体方便对点云以文件名和点云对象进行成对处理和管理点云，处理过程中可以同时接受多个点云文件的输入
struct PCD
{
    PointCloud::Ptr cloud;  //点云共享指针
    std::string f_name;   //文件名称

    PCD() : cloud (new PointCloud) {};
};

struct PCDComparator  //文件比较处理
{
    bool operator () (const PCD& p1, const PCD& p2)
    {
        return (p1.f_name < p2.f_name);
    }
};

// 以< x, y, z, curvature >形式定义一个新的点表示
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
    MyPointRepresentation ()
    {
        nr_dimensions_ = 4;    //定义点的维度
    }

    // 重载copyToFloatArray方法将点转化为四维数组 
    virtual void copyToFloatArray (const PointNormalT &p, float * out) const
    {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};

/*左视图用来显示未匹配的源和目标点云*/
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
    p->removePointCloud ("vp1_target");
    p->removePointCloud ("vp1_source");

    PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
    PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
    p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
    p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

    PCL_INFO ("Press q to begin the registration.\n");
    p-> spin();
}

/**右边显示配准后的源和目标点云*/
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
    p->removePointCloud ("source");
    p->removePointCloud ("target");

    PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");
    if (!tgt_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");

    PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature");
    if (!src_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");

    p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
    p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

    p->spinOnce();
}

/** \brief Load a set of PCD files that we want to register together
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  * \param models the resultant vector of point cloud datasets
  */

void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
    std::string extension (".pcd");
    // 第一个参数是命令本身，所以要从第二个参数开始解析
    for (int i = 1; i < argc; i++)
    {
        std::string fname = std::string (argv[i]);
        // PCD文件名至少为5个字符大小字符串（因为后缀名.pcd就已经占了四个字符位置）
        if (fname.size () <= extension.size ())
            continue;

        std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);
        //检查参数是否为一个pcd后缀的文件
        if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
        {
            //加载点云并保存在总体的点云列表中
            PCD m;
            m.f_name = argv[i];
            pcl::io::loadPCDFile (argv[i], *m.cloud);
            //从点云中移除NAN点也就是无效点
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);
            models.push_back (m);
        }
    }
}


/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  **/
//实现匹配，其中参数有输入一组需要配准的点云，以及是否需要进行下采样，其他参数输出配准后的点云以及变换矩阵
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
    // Downsample for consistency and speed
    // \note enable this for large datasets
    PointCloud::Ptr src (new PointCloud);   //存储滤波后的源点云
    PointCloud::Ptr tgt (new PointCloud);   //存储滤波后的目标点云
    pcl::VoxelGrid<PointT> grid;         //滤波处理对象

    if (downsample)
    {
        grid.setLeafSize (0.05, 0.05, 0.05);    //设置滤波时采用的体素大小
        grid.setInputCloud (cloud_src);
        grid.filter (*src);

        grid.setInputCloud (cloud_tgt);
        grid.filter (*tgt);
    }
    else
    {
        src = cloud_src;
        tgt = cloud_tgt;
    }

    // 计算表面的法向量和曲率
    PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

    pcl::NormalEstimation<PointT, PointNormalT> norm_est;     //点云法线估计对象
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch (30);

    norm_est.setInputCloud (src);
    norm_est.compute (*points_with_normals_src);
    pcl::copyPointCloud (*src, *points_with_normals_src);

    norm_est.setInputCloud (tgt);
    norm_est.compute (*points_with_normals_tgt);
    pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

    // Instantiate our custom point representation (defined above) ...
    MyPointRepresentation point_representation;
    // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    point_representation.setRescaleValues (alpha);

    // 配准
    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;   // 配准对象
    reg.setTransformationEpsilon (1e-6);   ///设置收敛判断条件，越小精度越大，收敛也越慢 
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm大于此值的点对不考虑
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance (0.1);  
    // 设置点表示
    reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

    reg.setInputSource (points_with_normals_src);   // 设置源点云
    reg.setInputTarget (points_with_normals_tgt);    // 设置目标点云

    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;

    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations (2);//设置最大的迭代次数，即每迭代两次就认为收敛，停止内部迭代
    for (int i = 0; i < 30; ++i)   //手动迭代，每手动迭代一次，在配准结果视口对迭代的最新结果进行刷新显示
    {
        PCL_INFO ("Iteration Nr. %d.\n", i);

        // 存储点云以便可视化
        points_with_normals_src = reg_result;
        reg.setInputSource (points_with_normals_src);
        reg.align (*reg_result);

        //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation () * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
            reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
    
        prev = reg.getLastIncrementalTransformation ();

        // visualize current state
        showCloudsRight(points_with_normals_tgt, points_with_normals_src);
    }
    // Get the transformation from target to source
    targetToSource = Ti.inverse();//deidao

    // Transform target back in source frame
    pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

    p->removePointCloud ("source");
    p->removePointCloud ("target");

    PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
    PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
    p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
    p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);

    PCL_INFO ("Press q to continue the registration.\n");
    p->spin ();

    p->removePointCloud ("source"); 
    p->removePointCloud ("target");

    //add the source to the transformed target
    *output += *cloud_src;
  
    final_transform = targetToSource;
}


int test_ICPPcl8(int argc, char** argv)
{
    // 存储管理所有打开的点云
  std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
  loadData (argc, argv, data);  // 加载所有点云到data

  // 检查输入
  if (data.empty ())
  {
    PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
    PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
    return (-1);
  }
  PCL_INFO ("Loaded %d datasets.", (int)data.size ());

  // 创建PCL可视化对象
  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
  p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);  //用左半窗口创建视口vp_1
  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);  //用右半窗口创建视口vp_2

  PointCloud::Ptr result (new PointCloud), source, target;
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;

  for (size_t i = 1; i < data.size (); ++i)   //循环处理所有点云
  {
    source = data[i-1].cloud;   //连续配准
    target = data[i].cloud;          // 相邻两组点云

    showCloudsLeft(source, target);  //可视化为配准的源和目标点云
      //调用子函数完成一组点云的配准，temp返回配准后两组点云在第一组点云坐标下的点云
    PointCloud::Ptr temp (new PointCloud);
    PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
    // pairTransform返回从目标点云target到source的变换矩阵
    pairAlign (source, target, temp, pairTransform, true);

    //把当前两两配准后的点云temp转化到全局坐标系下返回result
    pcl::transformPointCloud (*temp, *result, GlobalTransform);

    //用当前的两组点云之间的变换更新全局变换
    GlobalTransform = GlobalTransform * pairTransform;

    //保存转换到第一个点云坐标下的当前配准后的两组点云result到文件i.pcd
    std::stringstream ss;
    ss <<"../data/"<< i << ".pcd";
    pcl::io::savePCDFile (ss.str (), *result, true);

  }

}