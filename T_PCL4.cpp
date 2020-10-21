#include"T_PCL4.h"


int test_mainPCL4(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

     //生成并填充点云
    cloud->width  = 5;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size (); ++i)   //填充数据
    {
        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }

    std::cerr << "Cloud before filtering: " << std::endl;   //打印
    for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " " 
                        << cloud->points[i].y << " " 
                        << cloud->points[i].z << std::endl;
    

    /************************************************************************************
   创建直通滤波器的对象，设立参数，滤波字段名被设置为Z轴方向，可接受的范围为（0.0，1.0）
   即将点云中所有点的Z轴坐标不在该范围内的点过滤掉或保留，这里是过滤掉，由函数setFilterLimitsNegative设定
   ***********************************************************************************/
    // 设置滤波器对象
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);            //设置输入点云
    pass.setFilterFieldName ("z");         //设置过滤时所需要点云类型的Z字段
    pass.setFilterLimits (0.0, 1.0);        //设置在过滤字段的范围

    //pass.setFilterLimitsNegative (true);   //设置保留范围内还是过滤掉范围内
    pass.filter (*cloud_filtered);            //执行滤波，保存过滤结果在cloud_filtered

    std::cerr << "Cloud after filtering: " << std::endl;   //打印
    for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " " 
                        << cloud_filtered->points[i].y << " " 
                        << cloud_filtered->points[i].z << std::endl;

    return (0);
}


int test_VoxGridPCL4()
{
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

    //点云对象的读取
    pcl::PCDReader reader;
    reader.read ("../data/table_scene_lms400.pcd", *cloud);    //读取点云到cloud中
    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";

    /******************************************************************************
  创建一个叶大小为1cm的pcl::VoxelGrid滤波器，
**********************************************************************************/
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;  //创建滤波对象
    sor.setInputCloud (cloud);            //设置需要过滤的点云给滤波对象
    sor.setLeafSize (0.01f, 0.01f, 0.01f);  //设置滤波时创建的体素体积为1cm的立方体
    sor.filter (*cloud_filtered);           //执行滤波处理，存储输出

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

    pcl::PCDWriter writer;
    writer.write ("../data/table_scene_lms400_downsampled_ray.pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

}


int test_sorPCL4()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // 定义读取对象
    pcl::PCDReader reader;
     // 读取点云文件
    reader.read<pcl::PointXYZ> ("../data/table_scene_lms400.pcd", *cloud);
    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
   //个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   //创建滤波器对象
    sor.setInputCloud (cloud);                           //设置待滤波的点云
    sor.setMeanK (50);                               //设置在进行统计时考虑查询点临近点数
    sor.setStddevMulThresh (1.0);                      //设置判断是否为离群点的阀值
    sor.filter (*cloud_filtered);                    //存储

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("../data/table_scene_lms400_inliers_ray.pcd", *cloud_filtered, false);

    sor.setNegative (true);
    sor.filter (*cloud_filtered);
    writer.write<pcl::PointXYZ> ("../data/table_scene_lms400_outliers_ray.pcd", *cloud_filtered, false);

    return (0);
}
int test_proPCL4()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

    //创建点云并打印出来
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
    for(size_t i = 0; i < cloud->points.size (); ++i)
    {
        std::cerr << "    " << cloud->points[i].x << " " 
                        << cloud->points[i].y << " " 
                        << cloud->points[i].z << std::endl;
    }
    // 填充ModelCoefficients的值,使用ax+by+cz+d=0平面模型，其中 a=b=d=0,c=1 也就是X——Y平面
    //定义模型系数对象，并填充对应的数据
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    coefficients->values.resize (4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;

    // 创建ProjectInliers对象，使用ModelCoefficients作为投影对象的模型参数
    pcl::ProjectInliers<pcl::PointXYZ> proj;     //创建投影滤波对象
    proj.setModelType (pcl::SACMODEL_PLANE);      //设置对象对应的投影模型
    proj.setInputCloud (cloud);                   //设置输入点云
    proj.setModelCoefficients (coefficients);       //设置模型对应的系数
    proj.filter (*cloud_projected);                 //投影结果存储

    std::cerr << "Cloud after projection: " << std::endl;
    for (size_t i = 0; i < cloud_projected->points.size (); ++i)
    std::cerr << "    " << cloud_projected->points[i].x << " " 
                        << cloud_projected->points[i].y << " " 
                        << cloud_projected->points[i].z << std::endl;

    return (0);
}


int test_segPCL4()
{
    /**********************************************************************************************************
   从输入的.PCD 文件载入数据后，创建一个VOxelGrid滤波器对数据进行下采样，在这里进行下才样是为了加速处理过程，
   越少的点意味着分割循环中处理起来越快
   **********************************************************************************************************/
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), 
                             cloud_filtered_blob (new pcl::PCLPointCloud2);//申明滤波前后的点云

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>),cloud_p (new pcl::PointCloud<pcl::PointXYZ>),cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    // 读取PCD文件
    pcl::PCDReader reader;
    reader.read ("../data/table_scene_lms400.pcd", *cloud_blob);

    // 创建体素栅格下采样: 下采样的大小为1cm
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;  //体素栅格下采样对象
    sor.setInputCloud (cloud_blob);             //原始点云
    sor.setLeafSize (0.01f, 0.01f, 0.01f);    // 设置采样体素大小
    sor.filter (*cloud_filtered_blob);        //保存

    // 转换为模板点云
    pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

    // 保存下采样后的点云
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("../data/table_scene_lms400_downsampled_ray.pcd", *cloud_filtered, false);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    
    pcl::SACSegmentation<pcl::PointXYZ> seg;               //创建分割对象
    seg.setOptimizeCoefficients (true);                    //设置对估计模型参数进行优化处理
    seg.setModelType (pcl::SACMODEL_PLANE);                //设置分割模型类别
    seg.setMethodType (pcl::SAC_RANSAC);                   //设置用哪个随机参数估计方法
    seg.setMaxIterations (1000);                            //设置最大迭代次数
    seg.setDistanceThreshold (0.01);                      //判断是否为模型内点的距离阀值

    // 设置ExtractIndices的实际参数
    pcl::ExtractIndices<pcl::PointXYZ> extract;        //创建点云提取对象
    int i = 0, nr_points = (int) cloud_filtered->points.size ();
    // While 30% of the original cloud is still there
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
        // 为了处理点云包含的多个模型，在一个循环中执行该过程并在每次模型被提取后，保存剩余的点进行迭代
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

        std::stringstream ss;
        ss << "../data/table_scene_lms400_plane_" << i << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);   

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_filtered.swap (cloud_f);
        i++;
    }
    return (0);
}

int test_rcRemovalPCL4(int argc, char** argv)
{
    if (argc != 2)  //确保输入的参数
    {
        std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
        exit(0);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    //填充点云
    cloud->width  = 5;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }

    if (strcmp(argv[1], "-r") == 0)
    {
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;  //创建滤波器
        outrem.setInputCloud(cloud);    //设置输入点云
        outrem.setRadiusSearch(0.8);     //设置半径为0.8的范围内找临近点
        outrem.setMinNeighborsInRadius (2); //设置查询点的邻域点集数小于2的删除
        // apply filter
        outrem.filter (*cloud_filtered);     //执行条件滤波   在半径为0.8 在此半径内必须要有两个邻居点，此点才会保存
    }
    else if (strcmp(argv[1], "-c") == 0)
    {
        //创建条件限定的下的滤波器
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());   //创建条件定义对象

        //为条件定义对象添加比较算子
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));   //添加在Z字段上大于0的比较算子

        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));   //添加在Z字段上小于0.8的比较算子

        // 创建滤波器并用条件定义对象初始化
        pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
        condrem.setCondition (range_cond);               
        condrem.setInputCloud (cloud);                   //输入点云
        condrem.setKeepOrganized(true);               //设置保持点云的结构

        // 执行滤波
        condrem.filter (*cloud_filtered);  //大于0.0小于0.8这两个条件用于建立滤波器
    }
    else
    {
        std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
        exit(0);
    }

    std::cerr << "Cloud before filtering: " << std::endl;
    for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " "
                        << cloud->points[i].y << " "
                        << cloud->points[i].z << std::endl;
    std::cout<<"---"<<std::endl;

    // display pointcloud after filtering
    std::cerr << "Cloud after filtering: " << std::endl;
    for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " "
                        << cloud_filtered->points[i].y << " "
                        << cloud_filtered->points[i].z << std::endl;
    return (0);

}