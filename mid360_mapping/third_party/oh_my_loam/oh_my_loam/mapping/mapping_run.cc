#include "oh_my_loam/mapping/mapping_run.h"
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

namespace oh_my_loam {

MappingRun::MappingRun()
    : keyframe_count_(0),
      keyframe_dist_threshold_(0.5),
      keyframe_angle_threshold_(0.1),
      voxel_size_(0.1),
      scan_num_(16),
      enable_loop_closure_(true),
      loop_closure_detected_(false),
      map_needs_update_(true),
      last_keyframe_pose_(Eigen::Matrix4d::Identity()) {
  LOG(INFO) << "建图系统初始化";
  
  // 初始化降采样滤波器
  downsize_filter_map_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
  downsize_filter_scan_.setLeafSize(voxel_size_ * 0.5, voxel_size_ * 0.5, voxel_size_ * 0.5);
  
  // 初始化KD树
  kdtree_history_keyframes_ = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
  
  // 初始化结果
  result_.success = false;
  result_.map_updated = false;
  result_.map->clear();
  result_.trajectory->clear();
}

MappingRun::~MappingRun() {
  LOG(INFO) << "建图系统析构";
}

void MappingRun::SetParameters(float keyframe_dist_threshold, float keyframe_angle_threshold,
                           float voxel_size, int scan_num, bool enable_loop_closure) {
  keyframe_dist_threshold_ = keyframe_dist_threshold;
  keyframe_angle_threshold_ = keyframe_angle_threshold;
  voxel_size_ = voxel_size;
  scan_num_ = scan_num;
  enable_loop_closure_ = enable_loop_closure;
  
  // 更新降采样滤波器
  downsize_filter_map_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
  downsize_filter_scan_.setLeafSize(voxel_size_ * 0.5, voxel_size_ * 0.5, voxel_size_ * 0.5);
  
  LOG(INFO) << "建图参数设置: "
           << "关键帧距离阈值=" << keyframe_dist_threshold_ << ", "
           << "关键帧角度阈值=" << keyframe_angle_threshold_ << ", "
           << "体素大小=" << voxel_size_ << ", "
           << "扫描线数=" << scan_num_ << ", "
           << "回环检测=" << (enable_loop_closure_ ? "启用" : "禁用");
}

void MappingRun::Process(const PointCloudPtr& cloud, const OdometryResult& odometry_result) {
  if (!odometry_result.success || cloud->empty()) {
    LOG(WARNING) << "输入点云为空或里程计失败!";
    result_.success = false;
    return;
  }
  
  LOG(INFO) << "处理建图数据: " << cloud->size() << "个点";
  
  // 获取当前帧位姿
  Eigen::Matrix4d curr_pose = odometry_result.pose;
  
  // 优化当前帧位姿（对齐到局部地图）
  if (!keyframes_.empty()) {
    OptimizeLocalMap(cloud, odometry_result.feature_cloud, curr_pose);
  }
  
  // 判断是否添加关键帧
  bool add_keyframe = ShouldAddKeyFrame(curr_pose);
  
  if (add_keyframe) {
    // 创建点云副本
    PointCloudPtr cloud_copy(new PointCloud);
    *cloud_copy = *cloud;
    
    // 添加关键帧
    AddKeyFrame(cloud_copy, odometry_result.feature_cloud, curr_pose);
    
    // 检测回环
    if (enable_loop_closure_ && keyframes_.size() > 10) {
      loop_closure_detected_ = DetectAndProcessLoopClosure();
    }
    
    // 触发地图更新
    map_needs_update_ = true;
  }
  
  // 更新全局地图
  if (map_needs_update_) {
    UpdateGlobalMap();
    UpdateTrajectory();
    map_needs_update_ = false;
    result_.map_updated = true;
  } else {
    result_.map_updated = false;
  }
  
  // 成功完成处理
  result_.success = true;
}

MappingResult MappingRun::GetResult() const {
  return result_;
}

bool MappingRun::SaveMap(const std::string& filename) const {
  if (result_.map->empty()) {
    LOG(WARNING) << "地图为空，无法保存";
    return false;
  }
  
  try {
    LOG(INFO) << "保存地图到文件: " << filename;
    pcl::io::savePCDFileBinary(filename, *result_.map);
    
    // 保存轨迹
    std::string trajectory_filename = filename.substr(0, filename.find_last_of('.')) + "_trajectory.pcd";
    pcl::io::savePCDFileBinary(trajectory_filename, *result_.trajectory);
    
    return true;
  } catch (const std::exception& e) {
    LOG(ERROR) << "保存地图失败: " << e.what();
    return false;
  }
}

void MappingRun::Reset() {
  LOG(INFO) << "重置建图系统";
  keyframes_.clear();
  keyframe_count_ = 0;
  last_keyframe_pose_ = Eigen::Matrix4d::Identity();
  result_.map->clear();
  result_.trajectory->clear();
  loop_closure_detected_ = false;
  map_needs_update_ = true;
  result_.success = false;
  result_.map_updated = false;
}

bool MappingRun::ShouldAddKeyFrame(const Eigen::Matrix4d& curr_pose) const {
  if (keyframes_.empty()) {
    return true;  // 第一帧总是添加为关键帧
  }
  
  // 计算与上一关键帧的距离和角度差
  Eigen::Matrix4d diff = last_keyframe_pose_.inverse() * curr_pose;
  
  // 计算平移距离
  float translation_dist = diff.block<3, 1>(0, 3).norm();
  
  // 计算旋转角度 (以弧度为单位)
  Eigen::Matrix3d rotation_diff = diff.block<3, 3>(0, 0);
  Eigen::AngleAxisd angle_axis(rotation_diff);
  float rotation_angle = std::abs(angle_axis.angle());
  
  // 如果距离或角度超过阈值，添加新的关键帧
  return translation_dist > keyframe_dist_threshold_ || rotation_angle > keyframe_angle_threshold_;
}

void MappingRun::AddKeyFrame(const PointCloudPtr& cloud, const FeatureCloud& features,
                        const Eigen::Matrix4d& pose) {
  LOG(INFO) << "添加关键帧: #" << keyframe_count_;
  
  // 点云降采样
  PointCloudPtr downsampled_cloud(new PointCloud());
  downsize_filter_scan_.setInputCloud(cloud);
  downsize_filter_scan_.filter(*downsampled_cloud);
  
  // 创建关键帧
  KeyFrame keyframe(downsampled_cloud, features, pose, keyframe_count_);
  keyframes_.push_back(keyframe);
  
  // 更新最后一帧的位姿
  last_keyframe_pose_ = pose;
  
  // 增加关键帧计数
  keyframe_count_++;
  
  // 限制关键帧数量，保持一个滑动窗口
  const size_t max_keyframes = 100;
  if (keyframes_.size() > max_keyframes) {
    keyframes_.pop_front();
  }
}

void MappingRun::OptimizeLocalMap(const PointCloudPtr& cloud, const FeatureCloud& features, 
                              Eigen::Matrix4d& pose) {
  // 选择最近的几个关键帧作为局部地图
  const size_t num_local_frames = std::min<size_t>(5, keyframes_.size());
  
  if (num_local_frames < 1) return;
  
  // 构建局部地图
  PointCloudPtr local_map(new PointCloud());
  
  for (size_t i = keyframes_.size() - num_local_frames; i < keyframes_.size(); ++i) {
    // 转换关键帧到全局坐标系
    PointCloudPtr transformed_cloud(new PointCloud());
    pcl::transformPointCloud(*keyframes_[i].cloud, *transformed_cloud, keyframes_[i].pose.cast<float>());
    
    // 添加到局部地图
    *local_map += *transformed_cloud;
  }
  
  // 如果局部地图太大，进行降采样
  if (local_map->size() > 10000) {
    PointCloudPtr downsampled_local_map(new PointCloud());
    downsize_filter_map_.setInputCloud(local_map);
    downsize_filter_map_.filter(*downsampled_local_map);
    local_map = downsampled_local_map;
  }
  
  // 使用ICP优化当前帧位姿
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaxCorrespondenceDistance(voxel_size_ * 2);
  icp.setMaximumIterations(10);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  
  // 变换当前点云到初始位姿
  PointCloudPtr transformed_cloud(new PointCloud());
  pcl::transformPointCloud(*cloud, *transformed_cloud, pose.cast<float>());
  
  // 执行ICP
  icp.setInputSource(transformed_cloud);
  icp.setInputTarget(local_map);
  
  PointCloud aligned;
  icp.align(aligned);
  
  if (icp.hasConverged()) {
    // 获取ICP计算的变换矩阵
    Eigen::Matrix4f icp_transform = icp.getFinalTransformation();
    
    // 更新位姿
    pose = pose * icp_transform.cast<double>();
    
    LOG(INFO) << "局部地图优化成功，fitness score: " << icp.getFitnessScore();
  } else {
    LOG(WARNING) << "局部地图优化失败";
  }
}

bool MappingRun::DetectAndProcessLoopClosure() {
  if (keyframes_.size() < 10) {
    return false;
  }
  
  // 获取当前关键帧
  const KeyFrame& current_keyframe = keyframes_.back();
  
  // 只考虑一段距离之前的关键帧（避免检测到最近的帧）
  const int min_history = 20;
  
  if (keyframe_count_ < min_history + 10) {
    return false;
  }
  
  // 构建历史关键帧的中心点云（用于KD树搜索）
  PointCloudPtr keyframe_centers(new PointCloud());
  
  for (size_t i = 0; i < keyframes_.size() - min_history; ++i) {
    PointT center_point;
    const auto& pose = keyframes_[i].pose;
    center_point.x = pose(0, 3);
    center_point.y = pose(1, 3);
    center_point.z = pose(2, 3);
    center_point.intensity = i;  // 使用强度字段存储索引
    keyframe_centers->push_back(center_point);
  }
  
  if (keyframe_centers->empty()) {
    return false;
  }
  
  // 构建KD树
  kdtree_history_keyframes_->setInputCloud(keyframe_centers);
  
  // 查找当前关键帧位置附近的历史关键帧
  std::vector<int> index(1);
  std::vector<float> distance(1);
  
  PointT query_point;
  query_point.x = current_keyframe.pose(0, 3);
  query_point.y = current_keyframe.pose(1, 3);
  query_point.z = current_keyframe.pose(2, 3);
  
  // 在一定半径内搜索
  const float search_radius = 3.0;  // 3米范围内
  
  int found = kdtree_history_keyframes_->radiusSearch(query_point, search_radius, index, distance, 1);
  
  if (found > 0) {
    // 找到潜在的回环
    int history_idx = static_cast<int>(keyframe_centers->points[index[0]].intensity);
    const KeyFrame& loop_keyframe = keyframes_[history_idx];
    
    LOG(INFO) << "检测到潜在回环，当前帧: " << current_keyframe.id << ", 历史帧: " << loop_keyframe.id;
    
    // 使用ICP进行精确配准
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaxCorrespondenceDistance(voxel_size_ * 2);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-7);
    icp.setEuclideanFitnessEpsilon(1e-7);
    
    // 将当前关键帧点云转换到全局坐标系
    PointCloudPtr current_cloud_global(new PointCloud());
    pcl::transformPointCloud(*current_keyframe.cloud, *current_cloud_global, 
                           current_keyframe.pose.cast<float>());
    
    // 将历史关键帧点云转换到全局坐标系
    PointCloudPtr history_cloud_global(new PointCloud());
    pcl::transformPointCloud(*loop_keyframe.cloud, *history_cloud_global, 
                           loop_keyframe.pose.cast<float>());
    
    // 执行ICP
    icp.setInputSource(current_cloud_global);
    icp.setInputTarget(history_cloud_global);
    
    PointCloud aligned;
    icp.align(aligned);
    
    if (icp.hasConverged() && icp.getFitnessScore() < 0.3) {
      LOG(INFO) << "回环检测成功，fitness score: " << icp.getFitnessScore();
      
      // 在实际应用中，这里应该进行位姿图优化
      // 简化版本中，我们仅记录检测到了回环
      
      return true;
    }
  }
  
  return false;
}

void MappingRun::UpdateGlobalMap() {
  // 清空当前地图
  result_.map->clear();
  
  // 合并所有关键帧
  for (const auto& keyframe : keyframes_) {
    // 转换关键帧点云到全局坐标系
    PointCloudPtr transformed_cloud(new PointCloud());
    pcl::transformPointCloud(*keyframe.cloud, *transformed_cloud, keyframe.pose.cast<float>());
    
    // 添加到全局地图
    *result_.map += *transformed_cloud;
  }
  
  // 对全局地图进行降采样
  if (result_.map->size() > 1000) {
    PointCloudPtr downsampled_map(new PointCloud());
    downsize_filter_map_.setInputCloud(result_.map);
    downsize_filter_map_.filter(*downsampled_map);
    result_.map = downsampled_map;
  }
  
  LOG(INFO) << "全局地图更新完成，点数: " << result_.map->size();
}

void MappingRun::UpdateTrajectory() {
  // 清空轨迹点云
  result_.trajectory->clear();
  
  // 为每个关键帧添加一个轨迹点
  for (const auto& keyframe : keyframes_) {
    PointT trajectory_point;
    trajectory_point.x = keyframe.pose(0, 3);
    trajectory_point.y = keyframe.pose(1, 3);
    trajectory_point.z = keyframe.pose(2, 3);
    trajectory_point.intensity = keyframe.id;
    
    result_.trajectory->push_back(trajectory_point);
  }
  
  LOG(INFO) << "轨迹更新完成，点数: " << result_.trajectory->size();
}

}  // namespace oh_my_loam 