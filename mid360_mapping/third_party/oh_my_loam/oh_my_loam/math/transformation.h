#pragma once

#include <Eigen/Dense>
#include <string>

namespace oh_my_loam {

// 变换类封装了位姿表示和操作
class Transformation {
 public:
  // 默认构造函数：单位变换
  Transformation();

  // 从旋转矩阵和平移向量构造
  Transformation(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation);

  // 从变换矩阵构造
  explicit Transformation(const Eigen::Matrix4d& matrix);
  
  // 从四元数和平移向量构造
  Transformation(const Eigen::Quaterniond& q, const Eigen::Vector3d& t);
  
  // 从欧拉角(RPY)和平移向量构造
  static Transformation FromEuler(double roll, double pitch, double yaw,
                                double x, double y, double z);
  
  // 析构函数
  ~Transformation() = default;
  
  // 获取变换矩阵
  Eigen::Matrix4d Matrix() const;
  
  // 获取逆变换
  Transformation Inverse() const;
  
  // 变换点（新版本）
  Eigen::Vector3d Transform(const Eigen::Vector3d& point) const;
  
  // 获取平移部分
  Eigen::Vector3d Translation() const;
  
  // 获取旋转部分（四元数）
  Eigen::Quaterniond Rotation() const;
  
  // 获取旋转部分（旋转矩阵）
  Eigen::Matrix3d RotationMatrix() const;
  
  // 获取欧拉角 (Roll-Pitch-Yaw)
  Eigen::Vector3d Euler() const;
  
  // 设置平移
  void SetTranslation(const Eigen::Vector3d& translation);
  
  // 设置旋转（四元数）
  void SetRotation(const Eigen::Quaterniond& q);
  
  // 设置旋转（旋转矩阵）
  void SetRotation(const Eigen::Matrix3d& rotation);
  
  // 给平移叠加增量
  void AddTranslation(const Eigen::Vector3d& delta_translation);
  
  // 给旋转叠加增量（欧拉角形式）
  void AddRotation(const Eigen::Vector3d& delta_euler);
  
  // 以字符串形式输出
  std::string ToString() const;
  
  // 乘法运算符重载
  Transformation operator*(const Transformation& other) const;
  
 private:
  // 内部变换矩阵表示
  Eigen::Matrix4d matrix_;
};

}  // namespace oh_my_loam 