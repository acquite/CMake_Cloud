

#include <iostream>

// #include <opencv2/opencv.hpp>
// #include <torch/script.h>
#include <eigen3/Eigen/Dense>

#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <cmath>
#include "neuv_defs.hpp"

#include "T_PCL.h"
#include "T_Eigen.h"

using namespace std;

template <typename T>
static void G_Matrix_mulmatrix(T *p1, int iRow1, int iCol1, T *p2, int iRow2, int iCol2, T *p3)
{
     if (iRow1 != iRow2)
          return;

     //列优先
     //Eigen::Map< Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > map1(p1, iRow1, iCol1);
     //Eigen::Map< Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > map2(p2, iRow2, iCol2);
     //Eigen::Map< Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > map3(p3, iCol1, iCol2);

     //行优先
     Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> map1(p1, iRow1, iCol1);
     Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> map2(p2, iRow2, iCol2);
     Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> map3(p3, iCol1, iCol2);

     map3 = map1 * map2;
}

void G_TestEigen()
{
     //1.矩阵定义
     cout << "1.矩阵定义" << endl;
     Eigen::MatrixXd m(2, 2);
     Eigen::Vector3d vec3d;
     Eigen::Vector4d vec4d(1.0, 2.0, 3.0, 4.0);

     //2.动态矩阵，静态矩阵
     cout << "2.动态矩阵，静态矩阵" << endl;
     Eigen::MatrixXd matrixXd;
     Eigen::Matrix3d matrix3d;

     //3.矩阵元素的访问
     cout << "矩阵元素的访问" << endl;
     m(0, 0) = 1;
     m(0, 1) = 2;
     m(1, 0) = m(0, 0) + 3;
     m(1, 1) = m(0, 0) * m(0, 1);
     cout << m << endl
          << endl;

     //4.设置矩阵元素
     cout << "4.设置矩阵的元素" << endl;
     m << -1.5, 2.4, 6.7, 2.0;
     cout << m << endl
          << endl;

     int row = 4;
     int col = 5;
     Eigen::MatrixXf matrixXf(row, col);
     matrixXf << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20;
     std::cout << matrixXf << std::endl
               << endl;

     //Identity 单位矩阵
     matrixXf << Eigen::MatrixXf::Identity(row, col);
     cout << matrixXf << endl
          << endl;

     //5.重置矩阵大小
     cout << "5.重置矩阵大小" << endl;
     Eigen::MatrixXd matrixXd1(3, 3);
     m = matrixXd1;
     cout << m.rows() << "   " << m.cols() << std::endl
          << endl;

     //6。矩阵运算
     cout << "6.矩阵运算" << endl;
     m << 1, 2, 7, 3, 4, 8, 5, 6, 9;
     cout << m << endl
          << endl;

     matrixXd1 = Eigen::Matrix3d::Random();
     m += matrixXd1;
     cout << m << endl
          << endl;

     m *= 2;
     cout << m << endl
          << endl;

     cout << -m << endl
          << endl;

     cout << m << endl
          << endl;

     //7.求矩阵的转置，共轭矩阵，伴随矩阵
     cout << "7.求矩阵转置，共轭矩阵，伴随矩阵" << endl;
     cout << m.transpose() << endl
          << endl;

     cout << m.conjugate() << endl
          << endl;

     cout << m.adjoint() << endl
          << endl;

     cout << m << endl
          << endl;

     m.transposeInPlace();
     cout << m << endl
          << endl;

     //9.矩阵的块操作
     cout << "9.矩阵块操作" << endl;
     cout << m << endl
          << endl;

     cout << m.block(1, 1, 2, 2) << endl
          << endl;

     cout << m.block<1, 2>(0, 0) << endl
          << endl;

     cout << m.col(1) << endl
          << endl;

     cout << m.row(0) << endl
          << endl;

     //10.向量的块操作
     cout << "10.向量的块操作" << endl;
     Eigen::ArrayXf arrayXf(10);
     arrayXf << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
     cout << vec3d << endl
          << endl;

     cout << arrayXf << endl
          << endl;

     cout << arrayXf.head(5) << endl
          << endl;

     cout << arrayXf.tail(4) * 2 << endl
          << endl;

     //11. 求解矩阵的特征值和特征向量
     cout << "11.矩阵特征值和特征向量" << endl;
     Eigen::Matrix2f matrix2f;
     matrix2f << 1, 2, 3, 4;
     //形式就是这个形式，遇到高维的就直接用高维的替换掉matrix2f。
     Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigenSolver(matrix2f);
     if (eigenSolver.info() == Eigen::Success)
     {
          cout << "values" << endl;
          cout << eigenSolver.eigenvalues() << endl
               << endl;

          cout << eigenSolver.eigenvectors() << endl
               << endl;
     }

     //12.类Map及动态矩阵的使用
     cout << "12.类Map及动态矩阵的使用" << endl;
     int array1[4] = {1, 2, 3, 4};
     int array2[4] = {5, 6, 7, 8};
     int array3[4] = {0, 0, 0, 0};
     G_Matrix_mulmatrix(array1, 2, 2, array2, 2, 2, array3);
     for (int i = 0; i < 4; i++)
     {
          cout << array3[i] << endl;
     }
}

void G_TestPCL()
{
     pcl::PointCloud<pcl::PointXYZ> cloud;
     cloud.width = 5;
     cloud.height = 1;
     cloud.is_dense = false;

     cloud.points.resize(cloud.width * cloud.height);

     for (size_t i = 0; i < cloud.points.size(); ++i)
     {
          cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
          cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
          cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
     }

     pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
     std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;

     for (size_t i = 0; i < cloud.points.size(); ++i)
          std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
}

//-----------------------------------------------------NEUV 链接----------------------------------------------------
// 获取时间戳
double get_timestamp(void)
{
     struct timeval now;
     gettimeofday(&now, 0);
     return (double)(now.tv_sec) + (double)(now.tv_usec) / 1000000.0;
}

void showretval(int ret)
{
     if (ret == 0)
          return;
     std::cout << "ret:" << ret << std::endl;
}

// 对INeuvEvent虚类的实现
class myeventh : public neuvition::INeuvEvent
{
public:
     virtual void on_connect(int code, const char *msg)
     {
          if (code == 0)
          {
               std::cout << "[NEUVITION]| Connect..." << std::endl;
          }
     }

     virtual void on_disconnect(int code)
     {
          if (code == 0)
          {
               std::cout << "[NEUVITION]| Disconnect..." << std::endl;
          }
     }

     virtual void on_response(int code, enum neuvition::neuv_cmd_code cmd)
     {

          switch (cmd)
          {
          case neuvition::NEUV_CMD_START_SCAN:
          {
               if (code == 0)
               {
                    std::cout << "[NEUVITION]| Start scanning..." << std::endl;
               }
               break;
          }

          case neuvition::NEUV_CMD_STOP_SCAN:
          {
               if (code == 0)
               {
                    std::cout << "[NEUVITION]| Stop scanning..." << std::endl;
               }
               break;
          }

          case neuvition::NEUV_CMD_START_STREAM:
          {
               if (code == 0)
               {
                    std::cout << "[NEUVITION]| Start data streaming..." << std::endl;
               }
               break;
          }

          case neuvition::NEUV_CMD_STOP_STREAM:
          {
               if (code == 0)
               {
                    std::cout << "[NEUVITION]| Stop data streaming..." << std::endl;
               }
               break;
          }

          // 上传底层设备配置参数
          case neuvition::NEUV_CMD_GET_PARAMS:
          {
               if (code == 0)
               {
                    std::cout << "[NEUVITION]| Device parameters synced..." << std::endl;
               }
               break;
          }
          }
     }

     void on_framedata(int code, int64_t microsec, const neuvition::NeuvUnits &data, const neuvition::nvid_t &frame_id)
     {

          // 通过回调函数on_framedata()返回点云数据
          // 在此函数中进行点云处理
          std::cout << "[NEUVITION]| On framedata... | Size | " << data.size() << std::endl;

          if (data.size() == 0)
               return;

          for (neuvition::NeuvUnits::const_iterator iter = data.begin(); iter != data.end(); iter++)
          {
               const neuvition::NeuvUnit &np = (*iter);

               float x = np.x * 0.001;
               float y = np.y * 0.001;
               float z = np.z * 0.001;
               printf("%f, %f, %f ", x, y, z);
               // 如果设置摄像头开启，并开启图像获取，则RGB为真实颜色，否则为128
               uint8_t r = np.r;
               uint8_t g = np.g;
               uint8_t b = np.b;
               uint8_t intensity = np.intensity;
               uint32_t timestamp = np.timestamp;
          }
     }

     virtual void on_imudata(int code, int64_t microsec, const neuvition::NeuvUnits &data, const neuvition::ImuData &imu) {}

     virtual void on_pczdata(bool status) {}

     virtual void on_mjpgdata(int code, int64_t microsec, cv::Mat Mat) {}
};

int G_TestNeuvSdk()
{
     int ret = 0;

     // 可选: X/Y轴翻转
     ret = neuvition::set_flip_axis(false, true);
     showretval(ret);

     neuvition::INeuvEvent *phandler = new myeventh();
     // 建立与底层设备的连接
     ret = neuvition::setup_client("192.168.183.101", 6668, phandler /*event-handler*/, false /*auto-reconnect*/);
     sleep(1);

     // 查询状态
     if (neuvition::is_connected())
     {
          // 获取设备视场角、设备类型、俯仰角、位置参数
          double hfov = neuvition::get_hfov();
          double vfov = neuvition::get_vfov();
          // int device_type = neuvition::get_device_type();
          // double bias_y = 0.0; //设备俯仰角，默认为0
          // neuvition::LaserIncidentAngles laserangles = neuvition::get_laser_angles();
          // neuvition::NeuPosCor pos_cor = neuvition::get_poscor_params();
          // neuvition::compute_tof2xyz_table(hfov, vfov, bias_y, device_type, pos_cor);
          // neuvition::compute_tof2xyz_table_with_multi_lasers(laserangles, device_type, pos_cor);
          neuvition::compute_tof2xyz_table(hfov, vfov);
          neuvition::LaserIncidentAngles laserangles = neuvition::get_laser_angles();
          neuvition::compute_tof2xyz_table_with_multi_lasers(laserangles);

          // 设置激光功率%
          // 范围值：0~65%
          ret = neuvition::set_laser_power(60);
          usleep(20000); //20ms

          // 设置激光发射频率档位
          // 可选值：1:340KHZ 3:500KHZ
          ret = neuvition::set_laser_interval(1);
          usleep(20000); //20ms

          // 设置点云帧率FPS
          // 可选值：10,15
          ret = neuvition::set_frame_frequency(10);
          usleep(20000); //20ms

          // 可选: 设置摄像头开启，并开启图像获取
          // 如果不使用，直接注释
          ret = neuvition::set_camera_status(true);
          ret = neuvition::set_mjpg_curl(true);

          // 请求底层设备开始扫描
          ret = neuvition::start_scan();
          sleep(1); //1s

          // 请求底层设备开始送点云流
          ret = neuvition::start_stream();

          // 等待100秒,从底层设备获取TOF数据并回调
          // 等待时间可自行定义
          sleep(100);

          // 请求底层设备停止送点云流
          ret = neuvition::stop_stream();
          usleep(20000); //20ms

          // 请求底层设备停止扫描
          ret = neuvition::stop_scan();
          usleep(20000); //20ms

          // 断开与底层设备的连接
          ret = neuvition::teardown_client();
          usleep(200000); //20ms
     }

     delete phandler;
     return 0;
}

int main(int argc, char **argv)
{
     std::cout << "==================Start ===============\n";

#if 0
      std::cout << "==================Eigen ===============\n";
     G_TestEigen();
#endif

#if 0
     std::cout << "==================PCL ===============\n";
     G_TestPCL();
#endif

#if 0
     std::cout << "==================Lidar show ===============\n";
     G_TestNeuvSdk();
#endif

#if 0
    std::cout << "==================道路宽度 ===============\n";

        //路口宽度
    double lx1,ly1,lx2,ly2;
    // lx1=2541.60550041666;
    // ly1=01957.85104304797;
    // lx2=2559.35177971651;
    // ly2=1971.8080427442;

// //6793
//     lx1=2788.84389354714;
//     ly1=4130.3672794618;
//     lx2=2759.35042892102;
//     ly2=4124.21520914422;


//3724
    lx1=2932.96564102564;
    ly1=2311.46487179487;
    lx2=2974.13794871795;
    ly2=2358.04641025641;


    double dis=sqrt(pow((lx2-lx1),2)+pow((ly2-ly1),2));
    int idis=(int)dis;
    std::cout<<"距离参考："<<idis<<std::endl;
#endif

     // test3( argc,  argv);
     // test_passThrough();
     // test_Integral();

     // test_Narf(argc, argv);
     T_EigenRoation();

     return 0;
}
