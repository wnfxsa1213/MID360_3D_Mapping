#ifndef FASTLIOPROCESSOR_H
#define FASTLIOPROCESSOR_H

#include <QObject>
#include <QMutex>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include "LidarManager.h" // 添加LidarManager头文件以使用ImuData结构

// 前向声明
namespace fastlio2 {
    class FastLio2;
    struct FastLio2Options;
    struct FastLio2Result;
}

class FastLioProcessor : public QObject
{
    Q_OBJECT
public:
    explicit FastLioProcessor(QObject *parent = nullptr);
    ~FastLioProcessor();

    bool initialize(const QString &configFilePath);
    void reset();
    pcl::PointCloud<pcl::PointXYZI>::Ptr getGlobalMap();
    Eigen::Matrix4f getCurrentPose() const;
    bool saveMap(const QString &filePath);
    bool loadMap(const QString &filePath);

public slots:
    void processPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void processImuData(const ImuData& imuData);
    void processPointCloudWithTimestamp(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, uint64_t cloudTimestamp);

signals:
    void processFinished(pcl::PointCloud<pcl::PointXYZI>::Ptr processedCloud);
    void globalMapUpdated(pcl::PointCloud<pcl::PointXYZI>::Ptr newGlobalMap);
    void poseUpdated(const Eigen::Matrix4f &pose);
    void processorError(const QString &errorMessage);

private:
    bool initFastLio();
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsamplePointCloud(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float leaf_size);

    bool initialized;
    QString configFile;
    float mapResolution;
    bool useImuData;
    int fastLioThreadNum;
    
    // 全局地图和当前位姿
    pcl::PointCloud<pcl::PointXYZI>::Ptr globalMap;
    Eigen::Matrix4f currentPose;
    
    // 体素滤波器用于降采样
    pcl::VoxelGrid<pcl::PointXYZI> voxelFilter;
    
    // 互斥锁保护数据访问
    mutable QMutex mutex;
    
    // Fast-LIO2实例
    std::unique_ptr<fastlio2::FastLio2> fastLio;
    
    // IMU数据缓冲区
    std::vector<ImuData> imuBuffer;
    QMutex imuMutex;

    // 添加IMU静止状态共享变量
    bool lastImuIsStatic = false;
};

#endif // FASTLIOPROCESSOR_H 