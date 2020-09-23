#include "T_Eigen.h"
// #include <Eigen/src/Geometry/Quaternion.h>

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
    Eigen::Vector3d v1(1, 1, 0); //列向量
}