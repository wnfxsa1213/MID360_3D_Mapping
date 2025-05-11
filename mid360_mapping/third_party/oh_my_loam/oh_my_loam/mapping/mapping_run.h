#pragma once

#include <Eigen/Dense>
#include <memory>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <deque>
#include <unordered_map>
#include "../odometry/odometry_run.h"

namespace oh_my_loam {

// 建图结果结构体
struct MappingResult {
  bool success = false;
  bool map_updated = false;
  PointCloudPtr map = std::make_shared<PointCloud>();
  PointCloudPtr trajectory = std::make_shared<PointCloud>();
};

// 关键帧结构体
struct KeyFrame {
  PointCloudPtr cloud;
  FeatureCloud features;
  Eigen::Matrix4d pose;
  int id;
  
  KeyFrame(const PointCloudPtr& cloud_, const FeatureCloud& features_,
           const Eigen::Matrix4d& pose_, int id_)
    : cloud(cloud_), features(features_), pose(pose_), id(id_) {}
};

class MappingRun {
 public:
  MappingRun();
  ~MappingRun();

  // 设置参数
  void SetParameters(float keyframe_dist_threshold, float keyframe_angle_threshold,
                   float voxel_size, int scan_num, bool enable_loop_closure);

  // 处理点云数据与里程计结果
  void Process(const PointCloudPtr& cloud, const OdometryResult& odometry_result);
  
  // 获取建图结果
  MappingResult GetResult() const;
  
  // 保存地图到文件
  bool SaveMap(const std::string& filename) const;
  
  // 重置建图
  void Reset();

 private:
  // 判断是否添加关键帧
  bool ShouldAddKeyFrame(const Eigen::Matrix4d& curr_pose) const;
  
  // 将关键帧添加到地图中
  void AddKeyFrame(const PointCloudPtr& cloud, const FeatureCloud& features,
                  const Eigen::Matrix4d& pose);
  
  // 优化局部地图
  void OptimizeLocalMap(const PointCloudPtr& cloud, const FeatureCloud& features, 
                      Eigen::Matrix4d& pose);
  
  // 检测回环并优化
  bool DetectAndProcessLoopClosure();
  
  // 更新全局地图
  void UpdateGlobalMap();
  
  // 构建轨迹点云
  void UpdateTrajectory();
  
  // 成员变量
  MappingResult result_;
  
  // 关键帧管理
  std::deque<KeyFrame> keyframes_;
  Eigen::Matrix4d last_keyframe_pose_;
  int keyframe_count_;
  
  // 参数
  float keyframe_dist_threshold_;
  float keyframe_angle_threshold_;
  float voxel_size_;
  int scan_num_;
  bool enable_loop_closure_;
  
  // 数据结构
  pcl::VoxelGrid<PointT> downsize_filter_map_;
  pcl::VoxelGrid<PointT> downsize_filter_scan_;
  
  // 回环检测相关
  pcl::KdTreeFLANN<PointT>::Ptr kdtree_history_keyframes_;
  bool loop_closure_detected_;
  
  // 地图管理
  bool map_needs_update_;
};

}  // namespace oh_my_loam 