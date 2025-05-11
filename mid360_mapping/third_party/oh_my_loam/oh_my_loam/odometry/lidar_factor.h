#pragma once

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace oh_my_loam {

// 工具函数实现
template<typename T>
inline void AngleAxisRotatePoint(const T* angle_axis, const T* pt, T* result) {
    T theta2 = angle_axis[0]*angle_axis[0] + angle_axis[1]*angle_axis[1] + angle_axis[2]*angle_axis[2];
    if (theta2 > T(0.0)) {
        T theta = sqrt(theta2);
        T costheta = cos(theta);
        T sintheta = sin(theta);
        T theta_inv = T(1.0) / theta;

        T w[3];
        w[0] = angle_axis[0] * theta_inv;
        w[1] = angle_axis[1] * theta_inv;
        w[2] = angle_axis[2] * theta_inv;

        T dot = w[0]*pt[0] + w[1]*pt[1] + w[2]*pt[2];
        T cross[3];
        cross[0] = w[1]*pt[2] - w[2]*pt[1];
        cross[1] = w[2]*pt[0] - w[0]*pt[2];
        cross[2] = w[0]*pt[1] - w[1]*pt[0];

        for (int i = 0; i < 3; ++i) {
            result[i] = pt[i]*costheta + cross[i]*sintheta + w[i]*dot*(T(1.0)-costheta);
        }
    } else {
        result[0] = pt[0];
        result[1] = pt[1];
        result[2] = pt[2];
    }
}

/**
 * @brief 点到线残差因子
 */
class PointToLineFactor {
public:
  PointToLineFactor(const Eigen::Vector3d& curr_point,
                   const Eigen::Vector3d& line_point1,
                   const Eigen::Vector3d& line_point2,
                   double weight = 1.0)
      : curr_point_(curr_point),
        line_point1_(line_point1),
        line_point2_(line_point2),
        weight_(weight) {}

  template <typename T>
  bool operator()(const T* const rotation, const T* const translation, T* residual) const {
    // 类型转换
    Eigen::Matrix<T, 3, 1> cp = curr_point_.cast<T>();
    Eigen::Matrix<T, 3, 1> lp1 = line_point1_.cast<T>();
    Eigen::Matrix<T, 3, 1> lp2 = line_point2_.cast<T>();

    // 旋转和平移
    Eigen::Matrix<T, 3, 1> pt;
    AngleAxisRotatePoint(rotation, cp.data(), pt.data());
    pt += Eigen::Map<const Eigen::Matrix<T, 3, 1>>(translation);

    // 线方向
    Eigen::Matrix<T, 3, 1> line_dir = lp2 - lp1;
    line_dir.normalize();

    // 点到线的向量
    Eigen::Matrix<T, 3, 1> vec = pt - lp1;

    // 投影残差（点到线的垂直分量）
    Eigen::Matrix<T, 3, 1> cross = vec.cross(line_dir);
    residual[0] = weight_ * cross.norm();
    return true;
  }

private:
  const Eigen::Vector3d curr_point_;
  const Eigen::Vector3d line_point1_;
  const Eigen::Vector3d line_point2_;
  const double weight_;
};

/**
 * @brief 点到面残差因子
 */
class PointToPlaneFactor {
public:
  PointToPlaneFactor(const Eigen::Vector3d& curr_point,
                    const Eigen::Vector3d& plane_point1,
                    const Eigen::Vector3d& plane_point2,
                    const Eigen::Vector3d& plane_point3,
                    double weight = 1.0)
      : curr_point_(curr_point),
        plane_point1_(plane_point1),
        plane_point2_(plane_point2),
        plane_point3_(plane_point3),
        weight_(weight) {
    // 计算法向量
    Eigen::Vector3d v1 = plane_point2_ - plane_point1_;
    Eigen::Vector3d v2 = plane_point3_ - plane_point1_;
    plane_normal_ = v1.cross(v2).normalized();
  }

  template <typename T>
  bool operator()(const T* const rotation, const T* const translation, T* residual) const {
    Eigen::Matrix<T, 3, 1> cp = curr_point_.cast<T>();
    Eigen::Matrix<T, 3, 1> pp1 = plane_point1_.cast<T>();
    Eigen::Matrix<T, 3, 1> normal = plane_normal_.cast<T>();

    // 旋转和平移
    Eigen::Matrix<T, 3, 1> pt;
    AngleAxisRotatePoint(rotation, cp.data(), pt.data());
    pt += Eigen::Map<const Eigen::Matrix<T, 3, 1>>(translation);

    // 点到面的法向残差
    residual[0] = weight_ * normal.dot(pt - pp1);
    return true;
  }

private:
  const Eigen::Vector3d curr_point_;
  const Eigen::Vector3d plane_point1_;
  const Eigen::Vector3d plane_point2_;
  const Eigen::Vector3d plane_point3_;
  Eigen::Vector3d plane_normal_;
  const double weight_;
};

} // namespace oh_my_loam 