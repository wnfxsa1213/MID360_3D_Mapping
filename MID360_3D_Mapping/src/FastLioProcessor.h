#ifndef FASTLIOPROCESSOR_H
#define FASTLIOPROCESSOR_H

#include <QObject>
#include <QMutex>
#include <QString>
#include <QFileInfo>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include "../include/fastlio2/FastLio2.h"

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
    
signals:
    void processFinished(pcl::PointCloud<pcl::PointXYZI>::Ptr processedCloud);
    void globalMapUpdated(pcl::PointCloud<pcl::PointXYZI>::Ptr newGlobalMap);
    void poseUpdated(const Eigen::Matrix4f& pose);
    void processorError(const QString &errorMessage);
    
private:
    bool initFastLio();
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsamplePointCloud(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float leaf_size);
    
    QString configFile;
    bool initialized;
    float mapResolution;
    bool useImuData;
    int fastLioThreadNum;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr globalMap;
    Eigen::Matrix4f currentPose;
    pcl::VoxelGrid<pcl::PointXYZI> voxelFilter;
    
    std::unique_ptr<fastlio2::FastLio2> fastLio;
    mutable QMutex mutex;
};

#endif // FASTLIOPROCESSOR_H 