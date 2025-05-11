#pragma once

#include <Eigen/Dense>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "../feature/feature_extraction.h"

namespace oh_my_loam {

// 使用PCL原生点云类型，避免依赖问题
using PointCloud = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudPtr = typename PointCloud::Ptr;

// 里程计结果结构体
struct OdometryResult {
  bool success = false;
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d delta_pose = Eigen::Matrix4d::Identity();
  FeatureCloud feature_cloud;
};

class OdometryRun {
 public:
  OdometryRun();
  ~OdometryRun();

  // 设置参数
  void SetParameters(int num_scans, float edge_threshold, float surf_threshold,
                     float distance_threshold, float angle_threshold);

  // 处理点云数据
  void Process(const PointCloudPtr& cloud);
  
  // 获取里程计结果
  OdometryResult GetResult() const;

  // 重置里程计
  void Reset();

 private:
  // 优化当前帧与上一帧之间的变换
  void OptimizeTransformation(const FeatureCloud& curr_feature_cloud,
                              const FeatureCloud& prev_feature_cloud,
                              Eigen::Matrix4d& transform);

  // 寻找边缘特征的对应关系
  void FindEdgeCorrespondences(const PointCloudPtr& corner_points, 
                              const PointCloudPtr& prev_corner_points,
                              const Eigen::Matrix4d& transform);

  // 寻找平面特征的对应关系
  void FindSurfaceCorrespondences(const PointCloudPtr& surf_points,
                                 const PointCloudPtr& prev_surf_points,
                                 const Eigen::Matrix4d& transform);
  
  // 使用Ceres求解器优化位姿
  bool SolvePose(Eigen::Matrix4d& transform);
  
  // 将变换矩阵转换为旋转和平移向量
  void TransformToRotationAndTranslation(const Eigen::Matrix4d& transform,
                                        Eigen::Vector3d& rotation,
                                        Eigen::Vector3d& translation);

  // 将旋转和平移向量转换为变换矩阵
  void RotationAndTranslationToTransform(const Eigen::Vector3d& rotation,
                                        const Eigen::Vector3d& translation,
                                        Eigen::Matrix4d& transform);

  // 成员变量
  FeatureExtraction feature_extractor_;
  OdometryResult result_;
  
  // 上一帧的特征点云
  FeatureCloud prev_feature_cloud_;
  
  // 参数
  int num_scans_;
  float edge_threshold_;
  float surf_threshold_;
  float distance_threshold_;
  float angle_threshold_;
  
  // 是否为第一帧
  bool is_first_frame_;
  
  // 累积的位姿
  Eigen::Matrix4d accumulated_pose_;
};

}  // namespace oh_my_loam 