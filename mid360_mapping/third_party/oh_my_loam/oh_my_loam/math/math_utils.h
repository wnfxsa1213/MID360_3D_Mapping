#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace oh_my_loam {
namespace math {

// 旋转矩阵到欧拉角的转换 (RPY: Roll-Pitch-Yaw)
Eigen::Vector3d RotationMatrixToEulerAngles(const Eigen::Matrix3d& R);

// 欧拉角到旋转矩阵的转换
Eigen::Matrix3d EulerAnglesToRotationMatrix(const Eigen::Vector3d& euler);

// 四元数到旋转矩阵的转换
Eigen::Matrix3d QuaternionToRotationMatrix(const Eigen::Quaterniond& q);

// 旋转矩阵到四元数的转换
Eigen::Quaterniond RotationMatrixToQuaternion(const Eigen::Matrix3d& R);

// 变换矩阵插值
Eigen::Matrix4d InterpolateTransform(const Eigen::Matrix4d& transform1,
                                    const Eigen::Matrix4d& transform2,
                                    float ratio);

// 计算两个变换矩阵之间的相对变换
Eigen::Matrix4d RelativeTransform(const Eigen::Matrix4d& source,
                                 const Eigen::Matrix4d& target);

// 判断两个浮点数是否接近
inline bool IsClose(float a, float b, float epsilon = 1e-6) {
  return std::fabs(a - b) < epsilon;
}

// 角度标准化到[-π, π]
inline double NormalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

// 计算变换矩阵的逆
inline Eigen::Matrix4d InverseTransform(const Eigen::Matrix4d& transform) {
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  result.block<3, 3>(0, 0) = transform.block<3, 3>(0, 0).transpose();
  result.block<3, 1>(0, 3) = -result.block<3, 3>(0, 0) * transform.block<3, 1>(0, 3);
  return result;
}

// 计算变换矩阵中的平移部分
inline Eigen::Vector3d GetTranslation(const Eigen::Matrix4d& transform) {
  return transform.block<3, 1>(0, 3);
}

// 计算变换矩阵中的旋转部分
inline Eigen::Matrix3d GetRotation(const Eigen::Matrix4d& transform) {
  return transform.block<3, 3>(0, 0);
}

// 设置变换矩阵中的平移部分
inline void SetTranslation(Eigen::Matrix4d& transform, const Eigen::Vector3d& translation) {
  transform.block<3, 1>(0, 3) = translation;
}

// 设置变换矩阵中的旋转部分
inline void SetRotation(Eigen::Matrix4d& transform, const Eigen::Matrix3d& rotation) {
  transform.block<3, 3>(0, 0) = rotation;
}

// 创建变换矩阵
inline Eigen::Matrix4d CreateTransform(const Eigen::Matrix3d& rotation,
                                  const Eigen::Vector3d& translation) {
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = rotation;
  transform.block<3, 1>(0, 3) = translation;
  return transform;
}

}  // namespace math
}  // namespace oh_my_loam 