#include "OhMyLoamProcessor.h"
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDir>
#include <QDebug>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

// 修正oh_my_loam相关头文件路径
#include "../../../third_party/oh_my_loam/oh_my_loam/odometry/odometry_run.h"
#include "../../../third_party/oh_my_loam/oh_my_loam/mapping/mapping_run.h"
#include "../../../third_party/oh_my_loam/oh_my_loam/base/types.h"
#include "../../../third_party/oh_my_loam/common/log/log.h"

OhMyLoamProcessor::OhMyLoamProcessor(QObject *parent)
    : QObject(parent), initialized(false), mapResolution(0.2f), useImuData(false)
{
    // 初始化点云指针
    globalMap.reset(new pcl::PointCloud<pcl::PointXYZI>());
    
    // 初始化位姿
    currentPose = Eigen::Matrix4f::Identity();
    
    // 初始化体素滤波器
    voxelFilter.setLeafSize(mapResolution, mapResolution, mapResolution);
}

OhMyLoamProcessor::~OhMyLoamProcessor()
{
    // 清理资源
    if (initialized) {
        // 确保在析构时清理glog
        google::ShutdownGoogleLogging();
    }
}

bool OhMyLoamProcessor::initialize(const QString &configFilePath)
{
    QMutexLocker locker(&mutex);
    
    if (initialized) {
        emit processorError("Oh-My-LOAM已经初始化");
        return false;
    }
    
    configFile = configFilePath;
    
    // 检查配置文件是否存在
    QFile file(configFilePath);
    if (!file.exists()) {
        emit processorError("配置文件不存在: " + configFilePath);
        return false;
    }
    
    // 从配置文件中加载参数
    if (!file.open(QIODevice::ReadOnly)) {
        emit processorError("无法打开配置文件: " + configFilePath);
        return false;
    }
    
    QByteArray configData = file.readAll();
    file.close();
    
    QJsonDocument jsonDoc = QJsonDocument::fromJson(configData);
    if (jsonDoc.isNull() || !jsonDoc.isObject()) {
        emit processorError("配置文件格式错误");
        return false;
    }
    
    QJsonObject rootObj = jsonDoc.object();
    
    // 读取Oh-My-LOAM相关参数
    if (rootObj.contains("OhMyLoam")) {
        QJsonObject loamObj = rootObj["OhMyLoam"].toObject();
        
        if (loamObj.contains("map_resolution")) {
            mapResolution = static_cast<float>(loamObj["map_resolution"].toDouble());
        }
        
        if (loamObj.contains("use_imu_data")) {
            useImuData = loamObj["use_imu_data"].toBool();
        }
    }
    
    // 初始化Oh-My-LOAM
    if (!initOhMyLoam()) {
        emit processorError("Oh-My-LOAM初始化失败");
        return false;
    }
    
    // 更新体素滤波器参数
    voxelFilter.setLeafSize(mapResolution, mapResolution, mapResolution);
    
    initialized = true;
    return true;
}

void OhMyLoamProcessor::reset()
{
    QMutexLocker locker(&mutex);
    
    if (!initialized) {
        return;
    }
    
    // 重置全局地图
    globalMap->clear();
    
    // 重置位姿
    currentPose = Eigen::Matrix4f::Identity();
    
    // 重置Oh-My-LOAM相关状态
    if (odometry) {
        odometry->Reset();
    }
    
    if (mapping) {
        // 重新初始化mapping
        mapping.reset();
        mapping = std::make_unique<oh_my_loam::MappingRun>();
    }
    
    // 发送信号
    emit globalMapUpdated(globalMap);
    emit poseUpdated(currentPose);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr OhMyLoamProcessor::getGlobalMap()
{
    QMutexLocker locker(&mutex);
    if (!initialized || !globalMap) {
        return pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr mapCopy(new pcl::PointCloud<pcl::PointXYZI>());
    *mapCopy = *globalMap;
    return mapCopy;
}

Eigen::Matrix4f OhMyLoamProcessor::getCurrentPose() const
{
    QMutexLocker locker(&mutex);
    return currentPose;
}

bool OhMyLoamProcessor::saveMap(const QString &filePath)
{
    QMutexLocker locker(&mutex);
    
    if (!initialized || !globalMap || globalMap->empty()) {
        emit processorError("地图为空，无法保存");
        return false;
    }
    
    try {
        // 创建目录（如果不存在）
        QFileInfo fileInfo(filePath);
        QDir dir = fileInfo.dir();
        if (!dir.exists()) {
            dir.mkpath(".");
        }
        
        // 保存点云
        if (pcl::io::savePCDFile(filePath.toStdString(), *globalMap) == -1) {
            emit processorError("保存PCD文件失败");
            return false;
        }
        
        qDebug() << "保存地图成功: " << filePath;
        return true;
    } catch (const std::exception &e) {
        emit processorError(QString("保存地图异常: %1").arg(e.what()));
        return false;
    }
}

bool OhMyLoamProcessor::loadMap(const QString &filePath)
{
    QMutexLocker locker(&mutex);
    
    if (!initialized) {
        emit processorError("Oh-My-LOAM未初始化");
        return false;
    }
    
    try {
        // 检查文件是否存在
        if (!QFile::exists(filePath)) {
            emit processorError("地图文件不存在: " + filePath);
            return false;
        }
        
        // 加载点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr loadedMap(new pcl::PointCloud<pcl::PointXYZI>());
        if (pcl::io::loadPCDFile(filePath.toStdString(), *loadedMap) == -1) {
            emit processorError("加载PCD文件失败");
            return false;
        }
        
        // 更新全局地图
        *globalMap = *loadedMap;
        
        qDebug() << "加载地图成功: " << filePath << " 点云大小: " << globalMap->size();
        
        // 发送更新信号
        emit globalMapUpdated(globalMap);
        
        return true;
    } catch (const std::exception &e) {
        emit processorError(QString("加载地图异常: %1").arg(e.what()));
        return false;
    }
}

void OhMyLoamProcessor::processPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    if (!initialized || !odometry || !mapping) {
        emit processorError("Oh-My-LOAM未初始化，无法处理点云");
        return;
    }
    
    if (!cloud || cloud->empty()) {
        return;
    }
    
    try {
        QMutexLocker locker(&mutex);
        
        // 降采样点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = downsamplePointCloud(cloud, mapResolution);
        if (!filteredCloud || filteredCloud->empty()) {
            return;
        }
        
        // 转换为oh_my_loam点云格式
        oh_my_loam::PointCloudPtr loamCloud(new oh_my_loam::PointCloud);
        loamCloud->reserve(filteredCloud->size());
        
        for (const auto& pt : filteredCloud->points) {
            pcl::PointXYZI loamPoint;
            loamPoint.x = pt.x;
            loamPoint.y = pt.y;
            loamPoint.z = pt.z;
            loamPoint.intensity = pt.intensity;
            loamCloud->push_back(loamPoint);
        }
        
        // 使用Oh-My-LOAM处理点云
        // 1. 首先进行里程计计算
        odometry->Process(loamCloud);
        auto odometryResult = odometry->GetResult();
        
        if (!odometryResult.success) {
            emit processorError("里程计计算失败");
            return;
        }
        
        // 2. 然后进行建图
        mapping->Process(loamCloud, odometryResult);
        auto mappingResult = mapping->GetResult();
        
        // 3. 更新当前位姿
        Eigen::Matrix4f pose;
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                pose(i,j) = static_cast<float>(odometryResult.pose(i,j));
            }
        }
        currentPose = pose;
        
        // 4. 更新全局地图
        if (mappingResult.map_updated) {
            // 将新的地图点云转换并添加到globalMap
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZI>());
            transformedCloud->reserve(filteredCloud->size());
            
            // 使用当前位姿变换点云
            pcl::transformPointCloud(*filteredCloud, *transformedCloud, currentPose);
            
            // 将变换后的点云添加到全局地图
            *globalMap += *transformedCloud;
            
            // 对全局地图进行降采样以保持点云密度
            pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledMap(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::VoxelGrid<pcl::PointXYZI> voxelFilter;
            voxelFilter.setInputCloud(globalMap);
            voxelFilter.setLeafSize(mapResolution, mapResolution, mapResolution);
            voxelFilter.filter(*downsampledMap);
            *globalMap = *downsampledMap;
        }
        
        // 发送处理完成的点云
        emit processFinished(filteredCloud);
        
        // 发送更新的全局地图
        if (mappingResult.map_updated) {
            emit globalMapUpdated(globalMap);
        }
        
        // 更新位姿
        emit poseUpdated(currentPose);
        
    } catch (const std::exception& e) {
        emit processorError(QString("处理点云异常: %1").arg(e.what()));
    }
}

void OhMyLoamProcessor::processImuData(const ImuData& imuData)
{
    // Oh-My-LOAM默认不使用IMU数据，仅提供接口保持一致
    if (!initialized || !useImuData) {
        return;
    }
    
    // 如果配置为使用IMU，这里可以添加IMU数据处理逻辑
    qDebug() << "Oh-My-LOAM收到IMU数据，但当前未实现IMU处理";
}

void OhMyLoamProcessor::processPointCloudWithTimestamp(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, uint64_t cloudTimestamp)
{
    // 简单调用processPointCloud，时间戳未使用
    processPointCloud(cloud);
}

bool OhMyLoamProcessor::initOhMyLoam()
{
    try {
        // 创建OdometryRun实例
        odometry = std::make_unique<oh_my_loam::OdometryRun>();
        
        // 创建MappingRun实例
        mapping = std::make_unique<oh_my_loam::MappingRun>();
        
        // 设置日志级别 - 使用glog初始化
        ::google::InitGoogleLogging("OhMyLoam");
        // 输出到控制台
        FLAGS_logtostderr = true;
        // 设置日志级别
        FLAGS_minloglevel = google::GLOG_INFO;
        
        // 初始化成功
        return true;
    } catch (const std::exception& e) {
        emit processorError(QString("Oh-My-LOAM初始化失败: %1").arg(e.what()));
        return false;
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr OhMyLoamProcessor::downsamplePointCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float leaf_size)
{
    if (!cloud || cloud->empty()) {
        return pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    }
    
    // 优化点云下采样处理
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZI>());
    
    // 如果点云过小，不进行下采样
    if (cloud->size() < 1000) {
        *filteredCloud = *cloud;
        return filteredCloud;
    }
    
    // 配置体素滤波器
    pcl::VoxelGrid<pcl::PointXYZI> voxelFilter;
    voxelFilter.setInputCloud(cloud);
    voxelFilter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxelFilter.setDownsampleAllData(true); // 下采样所有点数据，包括强度等
    voxelFilter.setMinimumPointsNumberPerVoxel(1); // 每个体素最少1个点
    
    // 应用滤波器
    voxelFilter.filter(*filteredCloud);
    
    // 输出调试信息
    qDebug() << "点云下采样: 原始点数=" << cloud->size() 
             << "下采样后点数=" << filteredCloud->size()
             << "压缩比=" << (float)filteredCloud->size() / cloud->size() * 100.0f << "%";
    
    return filteredCloud;
} 