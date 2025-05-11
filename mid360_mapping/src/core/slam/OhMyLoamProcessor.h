#ifndef OHMYLOAMPROCESSOR_H
#define OHMYLOAMPROCESSOR_H

#include <QObject>
#include <QMutex>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include "LidarManager.h" // 使用ImuData结构

// oh_my_loam前向声明
namespace oh_my_loam {
    class OdometryRun;
    class MappingRun;
}

class OhMyLoamProcessor : public QObject
{
    Q_OBJECT
public:
    explicit OhMyLoamProcessor(QObject *parent = nullptr);
    ~OhMyLoamProcessor();

    bool initialize(const QString &configFilePath);
    void reset();
    pcl::PointCloud<pcl::PointXYZI>::Ptr getGlobalMap();
    Eigen::Matrix4f getCurrentPose() const;
    bool saveMap(const QString &filePath);
    bool loadMap(const QString &filePath);

public slots:
    void processPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void processImuData(const ImuData& imuData); // 提供接口但可能不使用
    void processPointCloudWithTimestamp(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, uint64_t cloudTimestamp); // 与FastLio接口保持一致

signals:
    void processFinished(pcl::PointCloud<pcl::PointXYZI>::Ptr processedCloud);
    void globalMapUpdated(pcl::PointCloud<pcl::PointXYZI>::Ptr newGlobalMap);
    void poseUpdated(const Eigen::Matrix4f &pose);
    void processorError(const QString &errorMessage);

private:
    bool initOhMyLoam();
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsamplePointCloud(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float leaf_size);
    
    bool initialized;
    QString configFile;
    float mapResolution;
    bool useImuData;
    
    // 全局地图和当前位姿
    pcl::PointCloud<pcl::PointXYZI>::Ptr globalMap;
    Eigen::Matrix4f currentPose;
    
    // 体素滤波器用于降采样
    pcl::VoxelGrid<pcl::PointXYZI> voxelFilter;
    
    // oh_my_loam实例
    std::unique_ptr<oh_my_loam::OdometryRun> odometry;
    std::unique_ptr<oh_my_loam::MappingRun> mapping;
    
    // 互斥锁保护数据访问
    mutable QMutex mutex;
};

#endif // OHMYLOAMPROCESSOR_H 