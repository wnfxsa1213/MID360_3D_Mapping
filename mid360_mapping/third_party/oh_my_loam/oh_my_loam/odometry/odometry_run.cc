#include "oh_my_loam/odometry/odometry_run.h"
#include <glog/logging.h>
#include <ceres/ceres.h>
#include <pcl/common/transforms.h>
#include "oh_my_loam/odometry/lidar_factor.h"

namespace oh_my_loam {

OdometryRun::OdometryRun()
    : num_scans_(16),
      edge_threshold_(0.1),
      surf_threshold_(0.1),
      distance_threshold_(5.0),
      angle_threshold_(0.1),
      is_first_frame_(true),
      accumulated_pose_(Eigen::Matrix4d::Identity()) {
  LOG(INFO) << "里程计系统初始化";
  
  // 初始化特征提取器
  feature_extractor_.SetParameters(num_scans_, edge_threshold_, surf_threshold_);
  
  // 初始化结果
  result_.success = false;
  result_.pose = Eigen::Matrix4d::Identity();
  result_.delta_pose = Eigen::Matrix4d::Identity();
}

OdometryRun::~OdometryRun() {
  LOG(INFO) << "里程计系统析构";
}

void OdometryRun::SetParameters(int num_scans, float edge_threshold, float surf_threshold,
                            float distance_threshold, float angle_threshold) {
  num_scans_ = num_scans;
  edge_threshold_ = edge_threshold;
  surf_threshold_ = surf_threshold;
  distance_threshold_ = distance_threshold;
  angle_threshold_ = angle_threshold;
  
  // 更新特征提取器参数
  feature_extractor_.SetParameters(num_scans_, edge_threshold_, surf_threshold_);
  
  LOG(INFO) << "里程计参数设置: " 
           << num_scans_ << "线, "
           << "边缘阈值=" << edge_threshold_ << ", "
           << "平面阈值=" << surf_threshold_ << ", "
           << "距离阈值=" << distance_threshold_ << ", "
           << "角度阈值=" << angle_threshold_;
}

void OdometryRun::Process(const PointCloudPtr& cloud) {
  if (cloud->empty()) {
    LOG(WARNING) << "输入点云为空!";
    result_.success = false;
    return;
  }
  
  LOG(INFO) << "处理点云数据: " << cloud->size() << "个点";

  // 提取当前帧特征
  FeatureCloud curr_feature_cloud = feature_extractor_.Extract(cloud);
  
  // 保存特征到结果中
  result_.feature_cloud = curr_feature_cloud;
  
  if (is_first_frame_) {
    LOG(INFO) << "第一帧，初始化位姿";
    is_first_frame_ = false;
    prev_feature_cloud_ = curr_feature_cloud;
    result_.success = true;
    result_.delta_pose = Eigen::Matrix4d::Identity();
    result_.pose = accumulated_pose_;
    return;
  }
  
  // 优化当前帧与上一帧之间的变换
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  OptimizeTransformation(curr_feature_cloud, prev_feature_cloud_, transform);
  
  // 更新累积位姿
  accumulated_pose_ = accumulated_pose_ * transform;
  
  // 保存结果
  result_.success = true;
  result_.delta_pose = transform;
  result_.pose = accumulated_pose_;
  
  // 将当前帧作为上一帧
  prev_feature_cloud_ = curr_feature_cloud;
  
  LOG(INFO) << "里程计计算成功，位移: " 
           << std::sqrt(transform(0,3)*transform(0,3) + transform(1,3)*transform(1,3) + transform(2,3)*transform(2,3));
}

OdometryResult OdometryRun::GetResult() const {
  return result_;
}

void OdometryRun::Reset() {
  LOG(INFO) << "重置里程计系统";
  is_first_frame_ = true;
  accumulated_pose_ = Eigen::Matrix4d::Identity();
  result_.success = false;
  result_.pose = Eigen::Matrix4d::Identity();
  result_.delta_pose = Eigen::Matrix4d::Identity();
}

void OdometryRun::OptimizeTransformation(const FeatureCloud& curr_feature_cloud,
                                     const FeatureCloud& prev_feature_cloud,
                                     Eigen::Matrix4d& transform) {
  // 初始变换为单位矩阵
  transform = Eigen::Matrix4d::Identity();
  
  // 最大迭代次数
  const int max_iterations = 10;
  
  for (int iter = 0; iter < max_iterations; ++iter) {
    // 寻找边缘特征的对应关系
    FindEdgeCorrespondences(curr_feature_cloud.corner_sharp,
                          prev_feature_cloud.corner_less_sharp,
                          transform);
    
    // 寻找平面特征的对应关系
    FindSurfaceCorrespondences(curr_feature_cloud.surface_flat,
                             prev_feature_cloud.surface_less_flat,
                             transform);
    
    // 使用Ceres求解器优化位姿
    bool success = SolvePose(transform);
    
    if (!success) {
      LOG(WARNING) << "位姿优化失败，使用上一次的结果";
      break;
    }
    
    LOG(INFO) << "迭代 " << iter << " 完成";
  }
}

void OdometryRun::FindEdgeCorrespondences(const PointCloudPtr& corner_points,
                                      const PointCloudPtr& prev_corner_points,
                                      const Eigen::Matrix4d& transform) {
  // LOAM算法中寻找边缘特征对应关系的实现
  // 简化实现，实际中需要使用kd-tree等加速搜索
  
  // 将当前帧转换到上一帧坐标系
  PointCloudPtr transformed_corner_points(new PointCloud());
  pcl::transformPointCloud(*corner_points, *transformed_corner_points, transform.cast<float>());
  
  // 对每个边缘特征点寻找最近的线段
  for (const auto& point : transformed_corner_points->points) {
    if (prev_corner_points->empty()) continue;
    
    // 简单实现：找到两个最近点组成线段
    int min_idx1 = -1;
    int min_idx2 = -1;
    float min_dist1 = distance_threshold_;
    float min_dist2 = distance_threshold_;
    
    for (size_t i = 0; i < prev_corner_points->size(); ++i) {
      float dist = std::sqrt(
          (point.x - prev_corner_points->points[i].x) * (point.x - prev_corner_points->points[i].x) +
          (point.y - prev_corner_points->points[i].y) * (point.y - prev_corner_points->points[i].y) +
          (point.z - prev_corner_points->points[i].z) * (point.z - prev_corner_points->points[i].z));
      
      if (dist < min_dist1) {
        min_dist2 = min_dist1;
        min_idx2 = min_idx1;
        min_dist1 = dist;
        min_idx1 = i;
      } else if (dist < min_dist2) {
        min_dist2 = dist;
        min_idx2 = i;
      }
    }
    
    // 找到了两个最近点，添加边缘特征约束
    if (min_idx1 >= 0 && min_idx2 >= 0) {
      // 在实际实现中，这里会添加到优化问题中
      // 简化版本中，我们只记录找到的对应关系
    }
  }
}

void OdometryRun::FindSurfaceCorrespondences(const PointCloudPtr& surf_points,
                                         const PointCloudPtr& prev_surf_points,
                                         const Eigen::Matrix4d& transform) {
  // LOAM算法中寻找平面特征对应关系的实现
  // 简化实现，实际中需要使用kd-tree等加速搜索
  
  // 将当前帧转换到上一帧坐标系
  PointCloudPtr transformed_surf_points(new PointCloud());
  pcl::transformPointCloud(*surf_points, *transformed_surf_points, transform.cast<float>());
  
  // 对每个平面特征点寻找最近的平面
  for (const auto& point : transformed_surf_points->points) {
    if (prev_surf_points->empty()) continue;
    
    // 简单实现：找到三个最近点构成平面
    int min_idx1 = -1;
    int min_idx2 = -1;
    int min_idx3 = -1;
    float min_dist1 = distance_threshold_;
    float min_dist2 = distance_threshold_;
    float min_dist3 = distance_threshold_;
    
    for (size_t i = 0; i < prev_surf_points->size(); ++i) {
      float dist = std::sqrt(
          (point.x - prev_surf_points->points[i].x) * (point.x - prev_surf_points->points[i].x) +
          (point.y - prev_surf_points->points[i].y) * (point.y - prev_surf_points->points[i].y) +
          (point.z - prev_surf_points->points[i].z) * (point.z - prev_surf_points->points[i].z));
      
      if (dist < min_dist1) {
        min_dist3 = min_dist2;
        min_idx3 = min_idx2;
        min_dist2 = min_dist1;
        min_idx2 = min_idx1;
        min_dist1 = dist;
        min_idx1 = i;
      } else if (dist < min_dist2) {
        min_dist3 = min_dist2;
        min_idx3 = min_idx2;
        min_dist2 = dist;
        min_idx2 = i;
      } else if (dist < min_dist3) {
        min_dist3 = dist;
        min_idx3 = i;
      }
    }
    
    // 找到了三个最近点，添加平面特征约束
    if (min_idx1 >= 0 && min_idx2 >= 0 && min_idx3 >= 0) {
      // 在实际实现中，这里会添加到优化问题中
      // 简化版本中，我们只记录找到的对应关系
    }
  }
}

bool OdometryRun::SolvePose(Eigen::Matrix4d& transform) {
  // 将变换矩阵转换为旋转和平移向量
  Eigen::Vector3d rotation;
  Eigen::Vector3d translation;
  TransformToRotationAndTranslation(transform, rotation, translation);
  
  // 构建Ceres优化问题
  ceres::Problem problem;
  ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);
  
  // 添加边缘特征约束
  // 在实际实现中，这里会根据FindEdgeCorrespondences找到的对应关系添加约束
  
  // 添加平面特征约束
  // 在实际实现中，这里会根据FindSurfaceCorrespondences找到的对应关系添加约束
  
  // 设置求解器选项
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 4;
  options.minimizer_progress_to_stdout = false;
  
  // 求解优化问题
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  
  // 检查求解结果
  if (summary.termination_type == ceres::CONVERGENCE ||
      summary.termination_type == ceres::NO_CONVERGENCE) {
    // 将优化结果转换回变换矩阵
    RotationAndTranslationToTransform(rotation, translation, transform);
    return true;
  } else {
    LOG(WARNING) << "求解失败: " << summary.BriefReport();
    return false;
  }
}

void OdometryRun::TransformToRotationAndTranslation(const Eigen::Matrix4d& transform,
                                                Eigen::Vector3d& rotation,
                                                Eigen::Vector3d& translation) {
  // 从变换矩阵中提取旋转矩阵
  Eigen::Matrix3d rotation_matrix = transform.block<3, 3>(0, 0);
  
  // 转换为欧拉角
  rotation = rotation_matrix.eulerAngles(0, 1, 2);
  
  // 提取平移向量
  translation = transform.block<3, 1>(0, 3);
}

void OdometryRun::RotationAndTranslationToTransform(const Eigen::Vector3d& rotation,
                                                const Eigen::Vector3d& translation,
                                                Eigen::Matrix4d& transform) {
  // 从欧拉角创建旋转矩阵
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix = Eigen::AngleAxisd(rotation(0), Eigen::Vector3d::UnitX())
                  * Eigen::AngleAxisd(rotation(1), Eigen::Vector3d::UnitY())
                  * Eigen::AngleAxisd(rotation(2), Eigen::Vector3d::UnitZ());
  
  // 填充变换矩阵
  transform.setIdentity();
  transform.block<3, 3>(0, 0) = rotation_matrix;
  transform.block<3, 1>(0, 3) = translation;
}

}  // namespace oh_my_loam 