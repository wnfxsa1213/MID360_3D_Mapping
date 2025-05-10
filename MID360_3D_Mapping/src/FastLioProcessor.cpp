#include "FastLioProcessor.h"
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDir>
#include <QDebug>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

FastLioProcessor::FastLioProcessor(QObject *parent)
    : QObject(parent), initialized(false), mapResolution(0.2f), useImuData(false), fastLioThreadNum(4)
{
    // 初始化点云指针
    globalMap.reset(new pcl::PointCloud<pcl::PointXYZI>());
    
    // 初始化位姿
    currentPose = Eigen::Matrix4f::Identity();
    
    // 初始化体素滤波器
    voxelFilter.setLeafSize(mapResolution, mapResolution, mapResolution);
}

FastLioProcessor::~FastLioProcessor()
{
    // 清理资源
}

bool FastLioProcessor::initialize(const QString &configFilePath)
{
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
    
    // 读取Fast-LIO相关参数
    if (rootObj.contains("FastLio")) {
        QJsonObject fastLioObj = rootObj["FastLio"].toObject();
        
        if (fastLioObj.contains("map_resolution")) {
            mapResolution = fastLioObj["map_resolution"].toDouble();
        }
        
        if (fastLioObj.contains("use_imu_data")) {
            useImuData = fastLioObj["use_imu_data"].toBool();
        }
        
        if (fastLioObj.contains("thread_num")) {
            fastLioThreadNum = fastLioObj["thread_num"].toInt();
        }
    }
    
    // 初始化Fast-LIO
    if (!initFastLio()) {
        emit processorError("Fast-LIO初始化失败");
        return false;
    }
    
    // 更新体素滤波器参数
    voxelFilter.setLeafSize(mapResolution, mapResolution, mapResolution);
    
    initialized = true;
    return true;
}

void FastLioProcessor::reset()
{
    QMutexLocker locker(&mutex);
    
    // 重置全局地图
    globalMap->clear();
    
    // 重置位姿
    currentPose = Eigen::Matrix4f::Identity();
    
    // 重置Fast-LIO相关状态
    if (fastLio) {
        fastLio->reset();
    }
    
    // 发送信号
    emit globalMapUpdated(globalMap);
    emit poseUpdated(currentPose);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr FastLioProcessor::getGlobalMap()
{
    QMutexLocker locker(&mutex);
    pcl::PointCloud<pcl::PointXYZI>::Ptr mapCopy(new pcl::PointCloud<pcl::PointXYZI>());
    *mapCopy = *globalMap;
    return mapCopy;
}

Eigen::Matrix4f FastLioProcessor::getCurrentPose() const
{
    // 使用QMutexLocker保护访问
    Eigen::Matrix4f poseCopy;
    {
        QMutexLocker locker(&mutex);
        poseCopy = currentPose;
    }
    return poseCopy;
}

bool FastLioProcessor::saveMap(const QString &filePath)
{
    QMutexLocker locker(&mutex);
    
    if (globalMap->empty()) {
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

bool FastLioProcessor::loadMap(const QString &filePath)
{
    try {
        QMutexLocker locker(&mutex);
        
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

void FastLioProcessor::processPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    if (!initialized || !fastLio) {
        emit processorError("Fast-LIO2未初始化，无法处理点云");
        return;
    }
    
    if (cloud->empty()) {
        return;
    }
    
    try {
        QMutexLocker locker(&mutex);
        
        // 1. 降采样点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = downsamplePointCloud(cloud, mapResolution);
        
        // 2. 使用Fast-LIO2处理点云
        fastlio2::FastLio2Result result = fastLio->processPointCloud(filteredCloud);
        
        // 3. 更新位姿
        currentPose = result.pose;
        
        // 4. 更新全局地图
        if (result.map_updated) {
            // 将新点云转换到全局坐标系
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(*result.cloud, *transformedCloud, currentPose);
            
            // 合并到全局地图
            *globalMap += *transformedCloud;
            
            // 对全局地图进行降采样
            pcl::PointCloud<pcl::PointXYZI>::Ptr filteredMap(new pcl::PointCloud<pcl::PointXYZI>());
            voxelFilter.setInputCloud(globalMap);
            voxelFilter.filter(*filteredMap);
            *globalMap = *filteredMap;
            
            // 添加调试输出
            qDebug() << "更新全局地图 - 当前点云点数:" << transformedCloud->size()
                     << "全局地图点数:" << globalMap->size();
        }
        
        // 5. 发送处理结果
        emit processFinished(filteredCloud);
        emit globalMapUpdated(globalMap);
        emit poseUpdated(currentPose);
        
    } catch (const std::exception& e) {
        emit processorError(QString("处理点云异常: %1").arg(e.what()));
    }
}

bool FastLioProcessor::initFastLio()
{
    try {
        // 初始化Fast-LIO2参数
        fastlio2::FastLio2Options options;
        options.num_threads = fastLioThreadNum;
        options.use_imu = useImuData;
        options.map_resolution = mapResolution;
        
        // 创建Fast-LIO2实例
        fastLio = std::make_unique<fastlio2::FastLio2>(options);
        
        // 初始化成功
        return true;
    } catch (const std::exception& e) {
        emit processorError(QString("Fast-LIO2初始化失败: %1").arg(e.what()));
        return false;
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr FastLioProcessor::downsamplePointCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float leaf_size)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGrid<pcl::PointXYZI> voxelFilter;
    voxelFilter.setInputCloud(cloud);
    voxelFilter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxelFilter.filter(*filteredCloud);
    return filteredCloud;
} 