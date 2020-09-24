#include "T_Eigen.h"
// #include <Eigen/src/Geometry/Quaternion.h>
using namespace std;

// 旋转向量->旋转矩阵
// 旋转向量->四元数
// 旋转向量->欧拉角
void T_EigenRoation()
{
     Eigen::AngleAxisd rotationVector(M_PI / 4, Eigen::Vector3d(0, 0, 1));
     Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
     rotationMatrix = rotationVector.toRotationMatrix();
     std::cout << "rotationMatrix\n"
               << rotationMatrix << std::endl;

     Eigen::Quaterniond q = Eigen::Quaterniond(rotationVector);
     std::cout << "rotation quaternion\n"
               << q.coeffs() << std::endl;

     Eigen::Vector3d eulerAngle = rotationVector.matrix().eulerAngles(0, 1, 2);
     std::cout << "eulerAngle roll pitch yaw\n"
               << 180 * eulerAngle / M_PI << std::endl;
}

void T_EigrnTest1()
{
     std::cout << "##----------------搞清旋转关系------------------##" << std::endl;
     Eigen::Vector3d v1(1, 1, 0); //列向量(1系下)
     Eigen::AngleAxisd angle_axis1(M_PI / 4, Eigen::Vector3d(0, 0, 1));
     Eigen::Vector3d rotated_v1 = angle_axis1.matrix().inverse() * v1;
     cout << "绕Z轴逆时针旋转45°（R12）：" << endl
          << angle_axis1.matrix() << endl;
     cout << "(1,1,0)旋转后：" << endl
          << rotated_v1.transpose() << endl;
     cout << "------------------------------------------" << endl;

     Eigen::Vector3d v2;
     v2 << 0, 1, 1;
     Eigen::AngleAxisd angle_axis2(M_PI / 4, Eigen::Vector3d(1, 0, 0));
     Eigen::Vector3d rotated_v2 = angle_axis2.matrix().inverse() * v2;
     cout << "绕X轴逆时针旋转45°（R12）:" << endl
          << angle_axis2.matrix() << endl;
     cout << "(0,1,1)旋转后：" << endl
          << rotated_v2.transpose() << endl;
     cout << "-----------------------------------------------" << endl;

     Eigen::Vector3d v3(0, 0, 0);
     v3.x() = 1;
     v3[2] = 1;
     Eigen::AngleAxisd angle_axis3(M_PI / 4, Eigen::Vector3d(0, 1, 0));
     Eigen::Vector3d rotated_v3 = angle_axis3.matrix().inverse() * v3;
     cout << "绕y轴逆时针旋转45°(R12):" << endl
          << angle_axis3.matrix() << endl;
     cout << "(1, 0, 1)旋转后:" << endl
          << rotated_v3.transpose() << endl;

     cout << "##-----------------常用数学运算-------------##" << endl;
     Eigen::Vector3d v4(1, 1, 0);
     cout << "(1,1,0)摸长：" << v4.norm() << endl;
     cout << "(1,1,0)单位向量：" << v4.normalized().transpose() << endl;

     Eigen::Vector3d v5(1, 0, 0), v6(0, 1, 0);
     cout << "(1,0,0)点乘（0,1,0）：" << v5.dot(v6) << endl;
     cout << "(1,0,0)叉乘（0,1,0）：" << v5.cross(v6).transpose() << endl;

     cout << "##---------使用块----------##" << endl;
     Eigen::Matrix<double, 4, 4> T12;
     T12.Identity();

     Eigen::AngleAxisd angle_axis(M_PI / 4, Eigen::Vector3d(0, 0, 1));
     Eigen::Matrix3d R12(angle_axis); //用角轴初始化旋转矩阵

     Eigen::Vector3d t12;
     t12.setOnes(); //各分量设为1
     // t12.setZero();//各分量设为0

     T12.block<3, 3>(0, 0) = R12;
     T12.block<3, 1>(0, 3) = t12;

     cout << "旋转R2：" << endl
          << T12.topLeftCorner(3, 3) << endl;
}
