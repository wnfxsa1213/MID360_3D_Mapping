#include "oh_my_loam/math/math_utils.h"
#include <glog/logging.h>

namespace oh_my_loam {
namespace math {

Eigen::Vector3d RotationMatrixToEulerAngles(const Eigen::Matrix3d& R) {
  // 使用Eigen内置函数计算欧拉角
  return R.eulerAngles(0, 1, 2);  // Roll, Pitch, Yaw
}

Eigen::Matrix3d EulerAnglesToRotationMatrix(const Eigen::Vector3d& euler) {
  Eigen::AngleAxisd roll_angle(euler(0), Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(euler(1), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(euler(2), Eigen::Vector3d::UnitZ());
  
  Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;
  return q.matrix();
}

Eigen::Matrix3d QuaternionToRotationMatrix(const Eigen::Quaterniond& q) {
  return q.normalized().toRotationMatrix();
}

Eigen::Quaterniond RotationMatrixToQuaternion(const Eigen::Matrix3d& R) {
  return Eigen::Quaterniond(R);
}

Eigen::Matrix4d InterpolateTransform(const Eigen::Matrix4d& transform1,
                                   const Eigen::Matrix4d& transform2,
                                   float ratio) {
  // 确保ratio在[0,1]范围内
  ratio = std::max(0.0f, std::min(1.0f, ratio));
  
  // 提取旋转部分
  Eigen::Matrix3d R1 = transform1.block<3, 3>(0, 0);
  Eigen::Matrix3d R2 = transform2.block<3, 3>(0, 0);
  
  // 转换为四元数，便于插值
  Eigen::Quaterniond q1(R1);
  Eigen::Quaterniond q2(R2);
  
  // 使用球面线性插值(slerp)
  Eigen::Quaterniond q = q1.slerp(ratio, q2);
  
  // 提取平移部分
  Eigen::Vector3d t1 = transform1.block<3, 1>(0, 3);
  Eigen::Vector3d t2 = transform2.block<3, 1>(0, 3);
  
  // 线性插值平移
  Eigen::Vector3d t = (1 - ratio) * t1 + ratio * t2;
  
  // 构建插值后的变换矩阵
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  result.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  result.block<3, 1>(0, 3) = t;
  
  return result;
}

Eigen::Matrix4d RelativeTransform(const Eigen::Matrix4d& source,
                                const Eigen::Matrix4d& target) {
  // target = source * relative
  // relative = source^-1 * target
  return InverseTransform(source) * target;
}

}  // namespace math
}  // namespace oh_my_loam 