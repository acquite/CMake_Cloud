#include"T_PCL2.h"
// https://blog.csdn.net/qq_34719188/article/details/79127174#PCL_840

void test_readPCD()
{
    //实例化的模板类PointCloud  每一个点的类型都设置为pcl::PointXYZ
	/*************************************************
	点PointXYZ类型对应的数据结构
	Structure PointXYZ{
	float x;
	float y;
	float z;
	};
	**************************************************/
    pcl::PointCloud<pcl::PointXYZ> cloud;
    // 创建点云  并设置适当的参数（width height is_dense）
	cloud.width = 5;
	cloud.height = 1;
	cloud.is_dense = false;  //不是稠密型的
    cloud.points.resize(cloud.width * cloud.height);  //点云总数大小

    //用随机数的值填充PointCloud点云对象 
    for (size_t i = 0; i < cloud.points.size(); ++i)
	{
        cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    //把PointCloud对象数据存储在 test_pcd.pcd文件中
	pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);

    //打印输出存储的点云数据
	std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;

    for (size_t i = 0; i < cloud.points.size(); ++i)
		std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

}

void test_loadPCD()
{
	//创建一个PointCloud<pcl::PointXYZ>    boost共享指针并进行实例化
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//打开点云文件
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("test_pcd.pcd", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return ;
	}

	//默认就是而二进制块读取转换为模块化的PointCLoud格式里pcl::PointXYZ作为点类型  然后打印出来
	std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from test_pcd.pcd with the following fields: "
		<< std::endl;
	
	for(size_t i=0;i<cloud->points.size();++i)
	{
		std::cout<<"	"<<cloud->points[i].x<<" "
						<<cloud->points[i].y<<"	"
						<<cloud->points[i].z<<std::endl;				
	}
}

void test_addCloud(int argc,char **argv)
{
	/***********************************************************
	这里一共有两种合并方式：
		第一种：点与点的合并
		第二种：点与向量的合并
	***********************************************************/
	//提示如果执行可执行文件输入两个参数 -f 或者-p
	if(argc!=2)
	{
		std::cerr << "please specify command line arg '-f' or '-p'" << std::endl;
		exit(0);
	}
	// argv[0]="-f";
	//argv[1] = "-p";  //点云 + 点云
	// argv[1] = "-f";  //点云 + 向量

	//申明三个pcl::PointXYZ点云数据类型，分别为cloud_a, cloud_b, cloud_c
	pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c;
	//存储进行连接时需要的Normal点云,Normal (float n_x, float n_y, float n_z)
	pcl::PointCloud<pcl::Normal> n_cloud_b;           //点与点合并的结果

	//存储连接XYZ与normal后的点云
	pcl::PointCloud<pcl::PointNormal> p_n_cloud_c;    //点与向量的合并结构

	// 创建点云数据
	//设置cloud_a的个数为5
	cloud_a.width = 5;
	cloud_a.height = cloud_b.height = n_cloud_b.height = 1; //设置都为无序点云
	cloud_a.points.resize(cloud_a.width * cloud_a.height); //总数
	if (strcmp(argv[1], "-p") == 0)   //判断是否为连接a+b=c(点云连接)
	{
		cloud_b.width = 3;   //这里代表点与点连接
		cloud_b.points.resize(cloud_b.width * cloud_b.height);
	}
	else
	{
		//这里是点与法向量连接
		n_cloud_b.width = 5; //如果是连接XYZ与normal则生成5个法线（字段间连接）
		n_cloud_b.points.resize(n_cloud_b.width * n_cloud_b.height);
	}

	//以下循环生成无序点云填充上面定义的两种类型的点云数据
	//先填充数据 cloud_a
	for (size_t i = 0; i < cloud_a.points.size(); ++i)
	{  //cloud_a产生三个点（每个点都有X Y Z 三个随机填充的值）
		cloud_a.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_a.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_a.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	if (strcmp(argv[1], "-p") == 0)
	{
		for (size_t i = 0; i < cloud_b.points.size(); ++i)
		{   //如果连接a+b=c，则填充cloud_b，用三个点作为xyz的数据
			cloud_b.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
			cloud_b.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
			cloud_b.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
		}
	}
	else
	{
		for (size_t i = 0; i < n_cloud_b.points.size(); ++i)
		{  //如果连接xyz+normal=xyznormal，则填充n_cloud_b，用5个点作为normal数据
			n_cloud_b.points[i].normal[0] = 1024 * rand() / (RAND_MAX + 1.0f);
			n_cloud_b.points[i].normal[1] = 1024 * rand() / (RAND_MAX + 1.0f);
			n_cloud_b.points[i].normal[2] = 1024 * rand() / (RAND_MAX + 1.0f);
		}
	}
	
	/***************************************************************
	到这里完成了以下工作：
		一、定义了连接点云会用到的5个点云对象：
			3个输入（cloud_a cloud_b 和n_cloud_b）
			2个输出（cloud_c  n_cloud_c）
		二、然后就是为两个输入点云cloud_a和 cloud_b或者cloud_a 和n_cloud_b填充数据
	****************************************************************/


	//输出Cloud A
	std::cerr << "Cloud A: " << std::endl;
	for (size_t i = 0; i < cloud_a.points.size(); ++i)
		std::cerr << "    " << cloud_a.points[i].x 
		<< " " << cloud_a.points[i].y 
		<< " " << cloud_a.points[i].z 
		<< std::endl;

	//输出Cloud B
	std::cerr << "Cloud B: " << std::endl;
	if (strcmp(argv[1], "-p") == 0)
		for (size_t i = 0; i < cloud_b.points.size(); ++i)  //输出 Cloud_b
			std::cerr << "    " << cloud_b.points[i].x 
			<< " " << cloud_b.points[i].y 
			<< " " << cloud_b.points[i].z 
			<< std::endl;
	else//输出n_Cloud_b
		for (size_t i = 0; i < n_cloud_b.points.size(); ++i)
			std::cerr << "    " << n_cloud_b.points[i].normal[0] 
			<< " " << n_cloud_b.points[i].normal[1] 
			<< " " << n_cloud_b.points[i].normal[2] 
			<< std::endl;
	
	// Copy the point cloud data
	if (strcmp(argv[1], "-p") == 0)
	{
		cloud_c=cloud_a;
		cloud_c+=cloud_b;//把cloud_a和cloud_b连接一起创建cloud_c  后输出
		std::cerr << "Cloud C 点云 + 点云: " << std::endl;
		for (size_t i = 0; i < cloud_c.points.size(); ++i)
			std::cerr << "    " << cloud_c.points[i].x << " " << cloud_c.points[i].y << " " << cloud_c.points[i].z << " " << std::endl;
	}
	else
	{
		//连接字段  把cloud_a和 n_cloud_b字段连接 一起创建 p_n_cloud_c)
		pcl::concatenateFields(cloud_a,n_cloud_b,p_n_cloud_c);
		std::cerr << "Cloud C 点云 + 向量: " << std::endl;
		for (size_t i = 0; i < p_n_cloud_c.points.size(); ++i)
			std::cerr << "    " <<
			p_n_cloud_c.points[i].x << " " << p_n_cloud_c.points[i].y << " " << p_n_cloud_c.points[i].z << " " <<
			p_n_cloud_c.points[i].normal[0] << " " << p_n_cloud_c.points[i].normal[1] << " " << p_n_cloud_c.points[i].normal[2] << std::endl;
	}
}

void test_gs()
{
	//创建一个点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = 1000;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	}

	//添加高斯噪声,范围（0-0.01）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudfiltered(new pcl::PointCloud<pcl::PointXYZ>());
	cloudfiltered->points.resize(cloud->points.size());
	cloudfiltered->header = cloud->header;
	cloudfiltered->width = cloud->width;
	cloudfiltered->height = cloud->height;

	boost::mt19937 rng;
	rng.seed(static_cast<unsigned int>(time(0)));
	boost::normal_distribution<> nd(0,0.01);
	boost::variate_generator<boost::mt19937&,boost::normal_distribution<>> var_nor(rng, nd);;

	for(size_t point_i=0;point_i<cloud->points.size();++point_i)
	{
		cloudfiltered->points[point_i].x=cloud->points[point_i].x+static_cast<float>(var_nor());
		cloudfiltered->points[point_i].y=cloud->points[point_i].y+static_cast<float>(var_nor());
		cloudfiltered->points[point_i].z=cloud->points[point_i].z+static_cast<float>(var_nor());
	}
}


void test_kdtree()
{
	srand (time (NULL));   //用系统时间初始化随机种子
	//创建一个PointCloud<pcl::PointXYZ>
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	// 随机点云生成
	cloud->width = 1000;             //此处点云数量
	cloud->height = 1;                //表示点云为无序点云
	cloud->points.resize (cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size (); ++i)   //循环填充点云数据
  	{
    cloud->points[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
  	}

	//创建KdTreeFLANN对象，并把创建的点云设置为输入,创建一个searchPoint变量作为查询点
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	//设置搜索空间
	kdtree.setInputCloud (cloud);
	//设置查询点并赋随机值
	pcl::PointXYZ searchPoint;
	searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
  	searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
  	searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);

	// K 临近搜索
	//创建一个整数（设置为10）和两个向量来存储搜索到的K近邻，两个向量中，一个存储搜索到查询点近邻的索引，另一个存储对应近邻的距离平方
	int K = 10;
	std::vector<int> pointIdxNKNSearch(K);//存储查询点近邻索引
	std::vector<float> pointNKNSquaredDistance(K);//存储近邻点对应距离平方

	//打印相关信息
	std::cout << "K nearest neighbor search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << " " << searchPoint.z
            << ") with K=" << K << std::endl;
	
	if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )  //执行K近邻搜索
	{
		 //打印所有近邻坐标
		for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
		{
			std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
                << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
                << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
                << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
		}
	}

	/**********************************************************************************
   下面的代码展示查找到给定的searchPoint的某一半径（随机产生）内所有近邻，重新定义两个向量
   pointIdxRadiusSearch  pointRadiusSquaredDistance来存储关于近邻的信息
   ********************************************************************************/
   // 半径 R内近邻搜索方法
   std::vector<int> pointIdxRadiusSearch;           //存储近邻索引
   std::vector<float> pointRadiusSquaredDistance;   //存储近邻对应距离的平方

	float radius = 256.0f * rand () / (RAND_MAX + 1.0f);   //随机的生成某一半径
	//打印输出
	std::cout << "Neighbors within radius search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << " " << searchPoint.z
            << ") with radius=" << radius << std::endl;
		
	if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )  //执行半径R内近邻搜索方法
	{
	for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
      std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
                << " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
                << " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
                << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;

	}
}

void test_coder()
{
	//创建一个新的SimpleOpenNIViewer  实例并调用他的run方法
	SimpleOpenNIViewer v;
	v.run();
}


int user_data_1;
void viewerOneOff_1 (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (0, 0, 0);       //设置背景颜色
    pcl::PointXYZ o;                                  //存储球的圆心位置
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);                  //添加圆球几何对象
    std::cout << "i only run once" << std::endl;
}


void viewerPsycho_1 (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);
    //FIXME: possible race condition here:
    user_data_1++;
}


void test_shape()
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);    //声明cloud 
    pcl::io::loadPCDFile ("../data/my_point_cloud.pcd", *cloud);         //加载点云文件
    pcl::visualization::CloudViewer viewer("Cloud Viewer");      //创建viewer对象


    //showCloud函数是同步的，在此处等待直到渲染显示为止
    viewer.showCloud(cloud);
    //该注册函数在可视化的时候只执行一次
    viewer.runOnVisualizationThreadOnce (viewerOneOff_1);
    //该注册函数在渲染输出时每次都调用
    viewer.runOnVisualizationThread (viewerPsycho_1);

    while (!viewer.wasStopped ())
    {
    //此处可以添加其他处理
    //FIXME: Note that this is running in a separate thread from viewerPsycho
    //and you should guard against race conditions yourself...
    user_data_1++;
    }
}


void test_octree()
{
	srand ((unsigned int) time (NULL));  //用系统时间初始化随机种子
	// 八叉树的分辨率，即体素的大小
  	float resolution = 32.0f;
	// 初始化空间变化检测对象
  	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZ> ); 
	//创建点云实例cloudA生成的点云数据用于建立八叉树octree对象

	// 为cloudA点云填充点数据
  	cloudA->width = 128;                      //设置点云cloudA的点数
  	cloudA->height = 1;                          //无序点
  	cloudA->points.resize (cloudA->width * cloudA->height);   //总数

	for (size_t i = 0; i < cloudA->points.size (); ++i)         //循环填充
	{
		cloudA->points[i].x = 64.0f * rand () / (RAND_MAX + 1.0f);
    	cloudA->points[i].y = 64.0f * rand () / (RAND_MAX + 1.0f);
    	cloudA->points[i].z = 64.0f * rand () / (RAND_MAX + 1.0f);
	}

	// 添加点云到八叉树中，构建八叉树
	octree.setInputCloud (cloudA);  //设置输入点云
	octree.addPointsFromInputCloud ();   //从输入点云构建八叉树

	/***********************************************************************************
    点云cloudA是参考点云用其建立的八叉树对象描述它的空间分布，octreePointCloudChangeDetector
    类继承自Octree2BufBae类，Octree2BufBae类允许同时在内存中保存和管理两个octree，另外它应用了内存池
    该机制能够重新利用已经分配了的节点对象，因此减少了在生成点云八叉树对象时昂贵的内存分配和释放操作
    通过访问 octree.switchBuffers ()重置八叉树 octree对象的缓冲区，但把之前的octree数据仍然保留在内存中
   ************************************************************************************/

	// 交换八叉树的缓冲，但是CloudA对应的八叉树结构仍然在内存中
	octree.switchBuffers ();

	//cloudB点云用于建立新的八叉树结构，与前一个cloudA对应的八叉树共享octree对象，同时在内存中驻留
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZ> );   //实例化点云对象cloudB
	// 为cloudB创建点云 
  	cloudB->width = 128;
  	cloudB->height = 1;
  	cloudB->points.resize (cloudB->width * cloudB->height);
	for (size_t i = 0; i < cloudB->points.size (); ++i)
  	{
    	cloudB->points[i].x = 64.0f * rand () / (RAND_MAX + 1.0f);
    	cloudB->points[i].y = 64.0f * rand () / (RAND_MAX + 1.0f);
    	cloudB->points[i].z = 64.0f * rand () / (RAND_MAX + 1.0f);
  	}
	// 添加cloudB到八叉树中
  	octree.setInputCloud (cloudB);
  	octree.addPointsFromInputCloud ();

	 /**************************************************************************************************************
  为了检索获取存在于couodB的点集R，此R并没有cloudA中的元素，可以调用getPointIndicesFromNewVoxels方法，通过探测两个八叉树之间
  体素的不同，它返回cloudB 中新加点的索引的向量，通过索引向量可以获取R点集  很明显这样就探测了cloudB相对于cloudA变化的点集，但是只能探测
  到在cloudA上增加的点集，二不能探测减少的
****************************************************************************************************************/
	std::vector<int> newPointIdxVector;  //存储新添加的索引的向量

	// 获取前一cloudA对应八叉树在cloudB对应在八叉树中没有的点集
	octree.getPointIndicesFromNewVoxels (newPointIdxVector);

	// 打印点集
	std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
	for (size_t i = 0; i < newPointIdxVector.size (); ++i)
	{
		std::cout << i << "# Index:" << newPointIdxVector[i]
              << "  Point:" << cloudB->points[newPointIdxVector[i]].x << " "
              << cloudB->points[newPointIdxVector[i]].y << " "
              << cloudB->points[newPointIdxVector[i]].z << std::endl;
	}
}


void test_change()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ> ());  
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_xyzrgba (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    pcl::copyPointCloud(*cloud_xyz, *cloud_xyzrgba);
}



//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------【2】------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	
}