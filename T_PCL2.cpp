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
    pcl::io::loadPCDFile ("../data/my_color.pcd", *cloud);         //加载点云文件
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
	// -----Open 3D viewer and add point cloud
	//创建视窗对象并给标题栏设置一个名称“3D Viewer”并将它设置为boost::shared_ptr智能共享指针，这样可以保证指针在程序中全局使用，而不引起内存错误
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//设置视窗的背景色，可以任意设置RGB的颜色，这里是设置为黑色
	viewer->setBackgroundColor(0, 0, 0);
	/*这是最重要的一行，我们将点云添加到视窗对象中，并定一个唯一的字符串作为ID 号，利用此字符串保证在其他成员中也能
	标志引用该点云，多次调用addPointCloud可以实现多个点云的添加，每调用一次就会创建一个新的ID号，如果想更新一个
	已经显示的点云，必须先调用removePointCloud（），并提供需要更新的点云ID 号*/
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	//用于改变显示点云的尺寸，可以利用该方法控制点云在视窗中的显示方法
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

	//查看复杂的点云，经常让人感到没有方向感，为了保持正确的坐标判断，需要显示坐标系统方向，可以通过使用X（红色）
	//Y（绿色 ）Z （蓝色）圆柱体代表坐标轴的显示方式来解决，圆柱体的大小可以通过scale参数来控制，本例中scale设置为1.0
	viewer->addCoordinateSystem(1.0);
	//通过设置照相机参数使得从默认的角度和方向观察点云
	viewer->initCameraParameters();
	return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	
	viewer->addCoordinateSystem(1.0);

	viewer->initCameraParameters();

	return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	viewer->setBackgroundColor(0, 0, 0);
	//创建一个自定义的颜色处理器PointCloudColorHandlerCustom对象，并设置颜色为纯绿色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);

	//addPointCloud<>()完成对颜色处理器对象的传递
	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	//实现对点云法线的显示
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals, 10, 0.05, "normals");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	/************************************************************************************************
	绘制形状的实例代码，绘制点之间的连线，
	*************************************************************************************************/

	viewer->addLine<pcl::PointXYZRGB>(cloud->points[0],	cloud->points[cloud->size() - 1], "line");
	//添加点云中第一个点为中心，半径为0.2的球体，同时可以自定义颜色
	viewer->addSphere(cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");


	//---------------------------------------
	//-----Add shapes at other locations添加绘制平面使用标准平面方程ax+by+cz+d=0来定义平面，这个平面以原点为中心，方向沿着Z方向-----
	//---------------------------------------
	pcl::ModelCoefficients coeffs;
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(1.0);
	coeffs.values.push_back(0.0);
	viewer->addPlane(coeffs, "plane");

	//添加锥形的参数
	coeffs.values.clear();
	coeffs.values.push_back(0.3);
	coeffs.values.push_back(0.3);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(1.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(5.0);
	viewer->addCone(coeffs, "cone");

	return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis(
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->initCameraParameters();
	//以上是创建视图的标准代码

	int v1(0);  //创建新的视口
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);  //4个参数分别是X轴的最小值，最大值，Y轴的最小值，最大值，取值0-1，v1是标识
	viewer->setBackgroundColor(0, 0, 0, v1);    //设置视口的背景颜色
	viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);  //添加一个标签区别其他窗口  利用RGB颜色着色器并添加点云到视口中
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud1", v1);


	//对第二视口做同样的操作，使得做创建的点云分布于右半窗口，将该视口背景赋值于灰色，以便明显区别，虽然添加同样的点云，给点云自定义颜色着色
	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, single_color, "sample cloud2", v2);

	//为所有视口设置属性，
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
	viewer->addCoordinateSystem(1.0);

	//添加法线  每个视图都有一组对应的法线
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals1, 10, 0.05, "normals1", v1);
	 //添加需要显示的点云法向。cloud为原始点云模型，normal为法向信息，10表示需要显示法向的点云间隔，即每10个点显示一次法向，0.0５表示法向长度
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals2, 10, 0.05, "normals2", v2);

	return (viewer);
}

unsigned int text_id = 0;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getKeySym() == "r" && event.keyDown())
	{
		std::cout << "r was pressed => removing all text" << std::endl;
		char str[512];
		for (unsigned int i = 0; i < text_id; ++i)
		{
			sprintf(str, "text#%03d", i);
			viewer->removeShape(str);
		}
		text_id = 0;
	}
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
	{
		std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

		char str[512];
		sprintf (str, "text#%03d", text_id ++);
		viewer->addText ("clicked here", event.getX (), event.getY (), str);
	}
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis ()
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
 	//以上是实例化视窗的标准代码
	
	viewer->addCoordinateSystem (1.0);
	//分别注册响应键盘和鼠标事件，keyboardEventOccurred  mouseEventOccurred回调函数，需要将boost::shared_ptr强制转换为void*
	viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());
	viewer->registerMouseCallback (mouseEventOccurred, (void*)viewer.get ());
	return (viewer);
}

void printUsage_1 (const char* progName)
{
	std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-s           Simple visualisation example\n"
            << "-r           RGB colour visualisation example\n"
            << "-c           Custom colour visualisation example\n"
            << "-n           Normals visualisation example\n"
            << "-a           Shapes visualisation example\n"
            << "-v           Viewports example\n"
            << "-i           Interaction Customization example\n"
            << "\n\n";
}

int test_MainPcl2(int argc,char** argv)
{
	// --------------------------------------
	// -----Parse Command Line Arguments-----
	// --------------------------------------
	if (pcl::console::find_argument (argc, argv, "-h") >= 0)
 	{
		printUsage_1(argv[0]);
   		return 0;
	}
	bool simple(false), rgb(false), custom_c(false), normals(false),shapes(false), viewports(false), interaction_customization(false);

	if (pcl::console::find_argument (argc, argv, "-s") >= 0)
  	{
		simple = true;
    	std::cout << "Simple visualisation example\n";
  	}
  	else if (pcl::console::find_argument (argc, argv, "-c") >= 0)
 	{
    	custom_c = true;
  	  	std::cout << "Custom colour visualisation example\n";
	}
  	else if (pcl::console::find_argument (argc, argv, "-r") >= 0)
  	{
    	rgb = true;
    	std::cout << "RGB colour visualisation example\n";
  	}
  	else if (pcl::console::find_argument (argc, argv, "-n") >= 0)
  	{
    	normals = true;
    	std::cout << "Normals visualisation example\n";
  	}
  	else if (pcl::console::find_argument (argc, argv, "-a") >= 0)
  	{
		shapes = true;
		std::cout << "Shapes visualisation example\n";
  	}
  	else if (pcl::console::find_argument (argc, argv, "-v") >= 0)
  	{
   		viewports = true;
    	std::cout << "Viewports example\n";
  	}
  	else if (pcl::console::find_argument (argc, argv, "-i") >= 0)
  	{
		interaction_customization = true;
    	std::cout << "Interaction Customization example\n";
  	}
  	else
  	{
   		printUsage_1(argv[0]);
    	return 0;
  	}

	// ------------------------------------
	// -----Create example point cloud-----
	// ------------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

	std::cout << "Genarating example point clouds.\n\n";

	// We're going to make an ellipse extruded along the z-axis. The colour for
	// the XYZRGB cloud will gradually go from red to green to blue.
	uint8_t r(255), g(15), b(15);
	for (float z(-1.0); z <= 1.0; z += 0.05)
  	{
		for (float angle(0.0); angle <= 360.0; angle += 5.0)
    	{
		// 圆柱面的参数方程为：
		// x = R*cos(θ);
		// y = R*sin(θ); 
		// z = z;
		// 其中 θ范围是[-2*PI, 2*PI), z的范围是(-∞，+∞）

		// 球面的参数方程是：
		// x = R*sin(θ)*cos(ψ); 
		// y = R*sin(θ)*sin(ψ); 
		// z = R*cos(θ);
		// 其中θ∈[0, PI), ψ∈[0, 2*PI)

			pcl::PointXYZ basic_point;
    		basic_point.x = 0.5 * cosf (pcl::deg2rad(angle));
     		basic_point.y = sinf (pcl::deg2rad(angle));
    		basic_point.z = z;
    		basic_cloud_ptr->points.push_back(basic_point);

			pcl::PointXYZRGB point;
    		point.x = basic_point.x;
    		point.y = basic_point.y;
    		point.z = basic_point.z;
    		uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
            static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      		point.rgb = *reinterpret_cast<float*>(&rgb);
      		point_cloud_ptr->points.push_back (point);
		}
		if(z<0.0)
		{
			r-=12;
			g+=12;
		}
		 else
    	{
    		g -= 12;
      		b += 12;
    	}
	}

	basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
	basic_cloud_ptr->height = 1;
	point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
	point_cloud_ptr->height = 1;

	// ----------------------------------------------------------------
	// -----Calculate surface normals with a search radius of 0.05-----
	// ----------------------------------------------------------------
	pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> ne;
	ne.setInputCloud (point_cloud_ptr);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
 
	ne.setSearchMethod (tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch (0.05);
	ne.compute (*cloud_normals1);


	// ---------------------------------------------------------------
	// -----Calculate surface normals with a search radius of 0.1-----
	// ---------------------------------------------------------------
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch (0.1);
	ne.compute (*cloud_normals2);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	if (simple)
  	{
		viewer = simpleVis(basic_cloud_ptr);
 	}
  	else if (rgb)
  	{
    	viewer = rgbVis(point_cloud_ptr);
  	}
  	else if (custom_c)
  	{
    	viewer = customColourVis(basic_cloud_ptr);
  	}
  	else if (normals)
  	{
    	viewer = normalsVis(point_cloud_ptr, cloud_normals2);
  	}
  	else if (shapes)
  	{
    	viewer = shapesVis(point_cloud_ptr);
 	}
 	 else if (viewports)
  	{
		viewer = viewportsVis(point_cloud_ptr, cloud_normals1, cloud_normals2);
 	}
	else if (interaction_customization)
	{
		viewer = interactionCustomizationVis();
	}

	//--------------------
	// -----Main loop-----
	//--------------------
	while (!viewer->wasStopped ())
  	{
    	viewer->spinOnce (100);
   	 	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  	}
}