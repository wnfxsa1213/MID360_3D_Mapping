#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <vector>

namespace oh_my_loam {

// 使用PCL原生点云类型
using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;
using PointCloudPtr = typename PointCloud::Ptr;

// 特征类型
enum FeatureType {
  CORNER_SHARP = 0,     // 尖锐边缘特征
  CORNER_LESS_SHARP,    // 次尖锐边缘特征
  SURFACE_FLAT,         // 平面特征
  SURFACE_LESS_FLAT     // 次平面特征
};

// 特征点云集合
struct FeatureCloud {
  PointCloudPtr corner_sharp;
  PointCloudPtr corner_less_sharp;
  PointCloudPtr surface_flat;
  PointCloudPtr surface_less_flat;
  
  FeatureCloud() {
    corner_sharp = std::make_shared<PointCloud>();
    corner_less_sharp = std::make_shared<PointCloud>();
    surface_flat = std::make_shared<PointCloud>();
    surface_less_flat = std::make_shared<PointCloud>();
  }
};

// 单个点的曲率计算结果
struct PointInfo {
  int idx;                // 点的索引
  float curvature;        // 曲率
  int scanID;             // 扫描线ID
  Eigen::Vector3f point;  // 点坐标
  
  PointInfo(int idx_, float curvature_, int scanID_, const Eigen::Vector3f& point_)
      : idx(idx_), curvature(curvature_), scanID(scanID_), point(point_) {}
      
  // 按曲率排序
  bool operator<(const PointInfo& other) const {
    return curvature < other.curvature;
  }
};

class FeatureExtraction {
 public:
  FeatureExtraction();
  ~FeatureExtraction();

  // 配置参数
  void SetParameters(int num_scans, float edge_threshold, float surf_threshold);
  
  // 提取特征点
  FeatureCloud Extract(const PointCloudPtr& cloud);

 private:
  // 计算点的曲率
  void ComputePointCurvature(const PointCloudPtr& cloud, std::vector<std::vector<PointInfo>>& scan_rings);
  
  // 提取边缘特征
  void ExtractEdgeFeatures(const PointCloudPtr& cloud, std::vector<std::vector<PointInfo>>& scan_rings, FeatureCloud& feature_cloud);
  
  // 提取平面特征
  void ExtractSurfaceFeatures(const PointCloudPtr& cloud, std::vector<std::vector<PointInfo>>& scan_rings, FeatureCloud& feature_cloud);

  // 参数
  int num_scans_;                 // 扫描线数量
  float edge_threshold_;          // 边缘特征阈值
  float surf_threshold_;          // 平面特征阈值
  int num_edge_features_;         // 每个扫描线上的边缘特征数量
  int num_surface_features_;      // 每个扫描线上的平面特征数量
};

}  // namespace oh_my_loam 