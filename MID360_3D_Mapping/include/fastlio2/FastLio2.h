#ifndef FASTLIO2_H
#define FASTLIO2_H

#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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
    FastLio2(const FastLio2Options& options) : options_(options) {}
    
    // 初始化
    bool initialize() {
        initialized_ = true;
        return true;
    }
    
    // 处理点云
    FastLio2Result processPointCloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud) {
        FastLio2Result result;
        
        // 简单的模拟处理
        result.map_updated = true;
        result.pose = current_pose_;
        result.cloud = cloud;
        
        // 更新当前位姿（简单模拟，实际应该使用点云配准结果）
        Eigen::Matrix4f delta = Eigen::Matrix4f::Identity();
        delta(0, 3) = 0.1f; // 简单的向X方向移动
        current_pose_ = current_pose_ * delta;
        
        return result;
    }
    
    // 处理IMU数据
    void processIMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr) {
        // 模拟IMU处理
    }
    
    // 是否已初始化
    bool isInitialized() const {
        return initialized_;
    }
    
    // 重置
    void reset() {
        current_pose_ = Eigen::Matrix4f::Identity();
    }
    
private:
    FastLio2Options options_;
    bool initialized_ = false;
    Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();
};

} // namespace fastlio2

#endif // FASTLIO2_H 