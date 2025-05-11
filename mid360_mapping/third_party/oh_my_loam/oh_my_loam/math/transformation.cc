#include "oh_my_loam/math/transformation.h"
#include "oh_my_loam/math/math_utils.h"
#include <iomanip>
#include <sstream>

namespace oh_my_loam {

Transformation::Transformation() : matrix_(Eigen::Matrix4d::Identity()) {}

Transformation::Transformation(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) {
  matrix_ = Eigen::Matrix4d::Identity();
  matrix_.block<3, 3>(0, 0) = rotation;
  matrix_.block<3, 1>(0, 3) = translation;
}

Transformation::Transformation(const Eigen::Matrix4d& matrix) : matrix_(matrix) {}

Transformation::Transformation(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
  matrix_ = Eigen::Matrix4d::Identity();
  matrix_.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  matrix_.block<3, 1>(0, 3) = t;
}

Transformation Transformation::FromEuler(double roll, double pitch, double yaw,
                                     double x, double y, double z) {
  Eigen::Vector3d euler(roll, pitch, yaw);
  Eigen::Matrix3d rotation = math::EulerAnglesToRotationMatrix(euler);
  Eigen::Vector3d translation(x, y, z);
  return Transformation(rotation, translation);
}

Eigen::Matrix4d Transformation::Matrix() const {
  return matrix_;
}

Transformation Transformation::Inverse() const {
  return Transformation(math::InverseTransform(matrix_));
}

Eigen::Vector3d Transformation::Transform(const Eigen::Vector3d& point) const {
  Eigen::Vector4d homogeneous_point(point.x(), point.y(), point.z(), 1.0);
  Eigen::Vector4d transformed = matrix_ * homogeneous_point;
  return transformed.head<3>() / transformed(3);
}

Eigen::Vector3d Transformation::Translation() const {
  return matrix_.block<3, 1>(0, 3);
}

Eigen::Quaterniond Transformation::Rotation() const {
  return Eigen::Quaterniond(matrix_.block<3, 3>(0, 0));
}

Eigen::Matrix3d Transformation::RotationMatrix() const {
  return matrix_.block<3, 3>(0, 0);
}

Eigen::Vector3d Transformation::Euler() const {
  return math::RotationMatrixToEulerAngles(matrix_.block<3, 3>(0, 0));
}

void Transformation::SetTranslation(const Eigen::Vector3d& translation) {
  matrix_.block<3, 1>(0, 3) = translation;
}

void Transformation::SetRotation(const Eigen::Quaterniond& q) {
  matrix_.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
}

void Transformation::SetRotation(const Eigen::Matrix3d& rotation) {
  matrix_.block<3, 3>(0, 0) = rotation;
}

void Transformation::AddTranslation(const Eigen::Vector3d& delta_translation) {
  matrix_.block<3, 1>(0, 3) += delta_translation;
}

void Transformation::AddRotation(const Eigen::Vector3d& delta_euler) {
  // 转换增量欧拉角为旋转矩阵
  Eigen::Matrix3d delta_rotation = math::EulerAnglesToRotationMatrix(delta_euler);
  
  // 应用到当前旋转
  matrix_.block<3, 3>(0, 0) = delta_rotation * matrix_.block<3, 3>(0, 0);
}

std::string Transformation::ToString() const {
  Eigen::Vector3d translation = Translation();
  Eigen::Vector3d euler = Euler() * 180.0 / M_PI;  // 转为角度
  
  std::stringstream ss;
  ss << std::fixed << std::setprecision(3);
  ss << "T=[" << translation.x() << ", " << translation.y() << ", " << translation.z() << "], ";
  ss << "RPY=[" << euler.x() << "°, " << euler.y() << "°, " << euler.z() << "°]";
  
  return ss.str();
}

Transformation Transformation::operator*(const Transformation& other) const {
  return Transformation(matrix_ * other.matrix_);
}

}  // namespace oh_my_loam 