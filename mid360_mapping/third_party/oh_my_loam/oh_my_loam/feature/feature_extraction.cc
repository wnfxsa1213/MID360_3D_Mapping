#include "oh_my_loam/feature/feature_extraction.h"
#include <algorithm>
#include <glog/logging.h>
#include <pcl/filters/voxel_grid.h>

namespace oh_my_loam {

FeatureExtraction::FeatureExtraction()
    : num_scans_(16),
      edge_threshold_(0.1),
      surf_threshold_(0.1),
      num_edge_features_(2),
      num_surface_features_(4) {
  LOG(INFO) << "特征提取器已初始化";
}

FeatureExtraction::~FeatureExtraction() {}

void FeatureExtraction::SetParameters(int num_scans, float edge_threshold, float surf_threshold) {
  num_scans_ = num_scans;
  edge_threshold_ = edge_threshold;
  surf_threshold_ = surf_threshold;
  
  // 根据扫描线数量调整特征点数量
  if (num_scans_ <= 16) {
    num_edge_features_ = 2;
    num_surface_features_ = 4;
  } else if (num_scans_ <= 32) {
    num_edge_features_ = 3;
    num_surface_features_ = 5;
  } else {
    num_edge_features_ = 4;
    num_surface_features_ = 8;
  }
  
  LOG(INFO) << "特征提取参数设置: " << num_scans_ << "线, "
           << "边缘阈值=" << edge_threshold_ << ", "
           << "平面阈值=" << surf_threshold_;
}

FeatureCloud FeatureExtraction::Extract(const PointCloudPtr& cloud) {
  FeatureCloud feature_cloud;
  
  if (cloud->empty()) {
    LOG(WARNING) << "输入点云为空!";
    return feature_cloud;
  }
  
  LOG(INFO) << "开始从 " << cloud->size() << " 个点中提取特征";
  
  // 将点按扫描线分组
  std::vector<std::vector<PointInfo>> scan_rings(num_scans_);
  
  // 计算每个点的曲率
  ComputePointCurvature(cloud, scan_rings);
  
  // 提取边缘特征
  ExtractEdgeFeatures(cloud, scan_rings, feature_cloud);
  
  // 提取平面特征
  ExtractSurfaceFeatures(cloud, scan_rings, feature_cloud);
  
  LOG(INFO) << "特征提取完成: " 
           << feature_cloud.corner_sharp->size() << " 个尖锐边缘特征, "
           << feature_cloud.corner_less_sharp->size() << " 个次尖锐边缘特征, "
           << feature_cloud.surface_flat->size() << " 个平面特征, "
           << feature_cloud.surface_less_flat->size() << " 个次平面特征";
           
  return feature_cloud;
}

void FeatureExtraction::ComputePointCurvature(const PointCloudPtr& cloud, 
                                          std::vector<std::vector<PointInfo>>& scan_rings) {
  const size_t cloud_size = cloud->size();
  
  // 为每一个点计算曲率
  for (size_t i = 5; i < cloud_size - 5; ++i) {
    // 计算扫描线ID (根据激光雷达垂直角度)
    float vertical_angle = std::atan2(cloud->points[i].z, 
                           std::sqrt(cloud->points[i].x * cloud->points[i].x + 
                                    cloud->points[i].y * cloud->points[i].y)) * 180 / M_PI;
    
    int scan_id = static_cast<int>((vertical_angle + 15) / 2.0 + 0.5);
    if (scan_id < 0 || scan_id >= num_scans_) {
      continue;
    }
    
    // 计算相邻点的曲率 (LOAM论文中的公式)
    float diff_x = 0;
    float diff_y = 0;
    float diff_z = 0;
    
    for (int j = -5; j <= 5; ++j) {
      if (j == 0) continue;
      diff_x += cloud->points[i + j].x;
      diff_y += cloud->points[i + j].y;
      diff_z += cloud->points[i + j].z;
    }
    
    diff_x = cloud->points[i].x * 11 - diff_x;
    diff_y = cloud->points[i].y * 11 - diff_y;
    diff_z = cloud->points[i].z * 11 - diff_z;
    
    float curvature = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
    
    // 保存点信息到对应的扫描线
    Eigen::Vector3f point_pos(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    scan_rings[scan_id].emplace_back(i, curvature, scan_id, point_pos);
  }
}

void FeatureExtraction::ExtractEdgeFeatures(const PointCloudPtr& cloud, 
                                       std::vector<std::vector<PointInfo>>& scan_rings, 
                                       FeatureCloud& feature_cloud) {
  for (int i = 0; i < num_scans_; ++i) {
    if (scan_rings[i].size() < 10) continue;
    
    // 按曲率排序
    std::sort(scan_rings[i].begin(), scan_rings[i].end());
    
    // 选择曲率最大的点作为尖锐边缘特征
    int picked_num = 0;
    for (int j = scan_rings[i].size() - 1; j >= 0; --j) {
      if (picked_num >= num_edge_features_) break;
      
      int idx = scan_rings[i][j].idx;
      if (scan_rings[i][j].curvature > edge_threshold_) {
        feature_cloud.corner_sharp->push_back(cloud->points[idx]);
        feature_cloud.corner_less_sharp->push_back(cloud->points[idx]);
        picked_num++;
      }
    }
    
    // 选择更多点作为次尖锐边缘特征
    picked_num = 0;
    for (int j = scan_rings[i].size() - 1; j >= 0; --j) {
      if (picked_num >= num_edge_features_ * 3) break;
      
      int idx = scan_rings[i][j].idx;
      if (scan_rings[i][j].curvature > edge_threshold_ * 0.5) {
        // 检查该点是否已经被选为尖锐特征
        bool already_selected = false;
        for (size_t k = 0; k < feature_cloud.corner_sharp->size(); ++k) {
          if (idx == feature_cloud.corner_sharp->points[k].intensity) {
            already_selected = true;
            break;
          }
        }
        
        if (!already_selected) {
          feature_cloud.corner_less_sharp->push_back(cloud->points[idx]);
          picked_num++;
        }
      }
    }
  }
}

void FeatureExtraction::ExtractSurfaceFeatures(const PointCloudPtr& cloud, 
                                          std::vector<std::vector<PointInfo>>& scan_rings, 
                                          FeatureCloud& feature_cloud) {
  for (int i = 0; i < num_scans_; ++i) {
    if (scan_rings[i].size() < 10) continue;
    
    // 选择曲率最小的点作为平面特征
    int picked_num = 0;
    for (size_t j = 0; j < scan_rings[i].size(); ++j) {
      if (picked_num >= num_surface_features_) break;
      
      int idx = scan_rings[i][j].idx;
      if (scan_rings[i][j].curvature < surf_threshold_) {
        feature_cloud.surface_flat->push_back(cloud->points[idx]);
        picked_num++;
      }
    }
    
    // 选择更多点作为次平面特征
    picked_num = 0;
    for (size_t j = 0; j < scan_rings[i].size(); ++j) {
      if (picked_num >= num_surface_features_ * 4) break;
      
      int idx = scan_rings[i][j].idx;
      if (scan_rings[i][j].curvature < surf_threshold_ * 2) {
        // 检查该点是否已经被选为平面特征
        bool already_selected = false;
        for (size_t k = 0; k < feature_cloud.surface_flat->size(); ++k) {
          if (idx == feature_cloud.surface_flat->points[k].intensity) {
            already_selected = true;
            break;
          }
        }
        
        if (!already_selected) {
          feature_cloud.surface_less_flat->push_back(cloud->points[idx]);
          picked_num++;
        }
      }
    }
    
    // 对次平面特征进行降采样
    pcl::VoxelGrid<PointT> down_size_filter;
    down_size_filter.setLeafSize(0.2, 0.2, 0.2);
    
    PointCloudPtr surface_less_flat_downsampled(new PointCloud());
    down_size_filter.setInputCloud(feature_cloud.surface_less_flat);
    down_size_filter.filter(*surface_less_flat_downsampled);
    
    feature_cloud.surface_less_flat = surface_less_flat_downsampled;
  }
}

}  // namespace oh_my_loam 