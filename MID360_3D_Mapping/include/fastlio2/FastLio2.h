#ifndef FASTLIO2_H
#define FASTLIO2_H

#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>

namespace fastlio2 {

struct FastLio2Options {
    int num_threads = 4;
    bool use_imu = true;
    float map_resolution = 0.2f;
};

struct FastLio2Result {
    bool map_updated = false;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud;
};

// Fast-LIO2算法的简化接口
class FastLio2 {
public:
    FastLio2(const FastLio2Options& options) : options_(options) {
        // 初始化ICP
        icp.setMaxCorrespondenceDistance(0.5);
        icp.setMaximumIterations(50);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        
        // 初始化体素滤波器
        voxel_filter.setLeafSize(options.map_resolution, 
                                options.map_resolution, 
                                options.map_resolution);
    }
    
    // 初始化
    bool initialize() {
        initialized_ = true;
        return true;
    }
    
    // 处理点云
    FastLio2Result processPointCloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud) {
        FastLio2Result result;
        
        if (!initialized_) {
            // 第一帧点云，直接作为地图
            if (last_cloud_ == nullptr) {
                last_cloud_ = cloud;
                result.map_updated = true;
                result.pose = current_pose_;
                result.cloud = cloud;
                return result;
            }
        }
        
        // 降采样当前点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        voxel_filter.setInputCloud(cloud);
        voxel_filter.filter(*filtered_cloud);
        
        // 使用ICP进行点云配准
        pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        icp.setInputSource(filtered_cloud);
        icp.setInputTarget(last_cloud_);
        icp.align(*aligned_cloud);
        
        if (icp.hasConverged()) {
            // 获取位姿变换
            Eigen::Matrix4f transform = icp.getFinalTransformation();
            
            // 更新当前位姿
            current_pose_ = current_pose_ * transform;
            
            // 更新结果
            result.map_updated = true;
            result.pose = current_pose_;
            result.cloud = aligned_cloud;
            
            // 更新上一帧点云
            last_cloud_ = filtered_cloud;
        } else {
            // ICP未收敛，保持原有点云
            result.map_updated = false;
            result.pose = current_pose_;
            result.cloud = cloud;
        }
        
        return result;
    }
    
    // 处理IMU数据
    void processIMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr) {
        // TODO: 实现IMU数据处理
    }
    
    // 是否已初始化
    bool isInitialized() const {
        return initialized_;
    }
    
    // 重置
    void reset() {
        current_pose_ = Eigen::Matrix4f::Identity();
        last_cloud_.reset();
        initialized_ = false;
    }
    
private:
    FastLio2Options options_;
    bool initialized_ = false;
    Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointXYZI>::Ptr last_cloud_;
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
};

} // namespace fastlio2

#endif // FASTLIO2_H 