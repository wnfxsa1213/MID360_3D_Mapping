#include "FastLioProcessor.h"
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDir>
#include <QDebug>
#include <deque>  // 用于IMU数据滤波
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include "../include/fastlio2/FastLio2.h"

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
        
        // 获取当前静止状态
        bool isStatic = false;
        {
            QMutexLocker imuLocker(&imuMutex);
            // 读取最近处理的IMU数据静止状态
            // 假设我们在IMU处理中设置了isStatic标志
            isStatic = lastImuIsStatic; // 需要在IMU处理中添加此变量
        }
        
        // 降采样点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = downsamplePointCloud(cloud, mapResolution);
        
        // 使用Fast-LIO2处理点云
        fastlio2::FastLio2Result result = fastLio->processPointCloud(filteredCloud);
        
        // 更新位姿
        currentPose = result.pose;
        
        // 更新全局地图 - 静止状态下仍然更新当前帧到系统
        emit processFinished(filteredCloud); // 始终发送当前帧供显示
        
        // 静止状态下，可以选择不更新全局地图，但保持当前帧显示
        if (!isStatic && result.map_updated) {
            // 非静止状态下更新全局地图
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
            
            // 发送更新信号
            emit globalMapUpdated(globalMap);
        }
        
        // 始终更新位姿
        emit poseUpdated(currentPose);
        
    } catch (const std::exception& e) {
        emit processorError(QString("处理点云异常: %1").arg(e.what()));
    }
}

void FastLioProcessor::processImuData(const ImuData& imuData)
{
    if (!initialized) {
        qDebug() << "Fast-LIO2未初始化，忽略IMU数据";
        return; // 如果未初始化，则直接返回
    }
    
    if (!fastLio) {
        qDebug() << "Fast-LIO2实例为空，忽略IMU数据";
        return; // Fast-LIO实例不存在，直接返回
    }
    
    if (!useImuData) {
        // 仅在调试模式下输出日志，避免大量无用日志
        // qDebug() << "IMU数据未启用，忽略数据";
        return; // 如果未启用IMU，直接返回
    }

    try {
        // 数据有效性检查
        if (std::isnan(imuData.acc_x) || std::isnan(imuData.acc_y) || std::isnan(imuData.acc_z) ||
            std::isnan(imuData.gyro_x) || std::isnan(imuData.gyro_y) || std::isnan(imuData.gyro_z)) {
            qWarning() << "IMU数据包含NaN值，已忽略";
            return;
        }
        
        // 对IMU数据进行滤波处理
        static const int FILTER_WINDOW_SIZE = 10; // 增大滤波窗口
        static std::deque<Eigen::Vector3d> accBuffer;
        static std::deque<Eigen::Vector3d> gyroBuffer;
        static Eigen::Vector3d lastAcc = Eigen::Vector3d::Zero();
        static Eigen::Vector3d lastGyr = Eigen::Vector3d::Zero();
        
        // 将新数据添加到缓冲区
        Eigen::Vector3d rawAcc(imuData.acc_x, imuData.acc_y, imuData.acc_z);
        Eigen::Vector3d rawGyr(imuData.gyro_x, imuData.gyro_y, imuData.gyro_z);
        
        accBuffer.push_back(rawAcc);
        gyroBuffer.push_back(rawGyr);
        
        // 维持缓冲区大小
        if (accBuffer.size() > FILTER_WINDOW_SIZE) {
            accBuffer.pop_front();
        }
        if (gyroBuffer.size() > FILTER_WINDOW_SIZE) {
            gyroBuffer.pop_front();
        }
        
        // 如果缓冲区尚未填满，使用原始数据
        if (accBuffer.size() < FILTER_WINDOW_SIZE) {
            // 添加IMU数据到缓冲区
            {
                QMutexLocker locker(&imuMutex);
                imuBuffer.push_back(imuData);
                const size_t MAX_IMU_BUFFER_SIZE = 1000;
                if (imuBuffer.size() > MAX_IMU_BUFFER_SIZE) {
                    imuBuffer.erase(imuBuffer.begin(), imuBuffer.begin() + (imuBuffer.size() - MAX_IMU_BUFFER_SIZE));
                }
            }
            fastLio->processIMU(imuData.timestamp, rawAcc, rawGyr);
            if (imuData.timestamp % 1000000 == 0) {
                qDebug() << "IMU数据已处理(原始): timestamp=" << imuData.timestamp
                       << " acc=(" << rawAcc.x() << "," << rawAcc.y() << "," << rawAcc.z() << ")"
                       << " gyr=(" << rawGyr.x() << "," << rawGyr.y() << "," << rawGyr.z() << ")";
            }
            lastAcc = rawAcc;
            lastGyr = rawGyr;
            return;
        }
        
        // 计算移动平均
        Eigen::Vector3d filteredAcc = Eigen::Vector3d::Zero();
        Eigen::Vector3d filteredGyr = Eigen::Vector3d::Zero();
        for (const auto& acc : accBuffer) {
            filteredAcc += acc;
        }
        for (const auto& gyr : gyroBuffer) {
            filteredGyr += gyr;
        }
        filteredAcc /= accBuffer.size();
        filteredGyr /= gyroBuffer.size();
        
        // --- IMU静止检测与零速校正 ---
        double accDelta = (filteredAcc - lastAcc).norm();
        double gyrDelta = (filteredGyr - lastGyr).norm();
        const double ACC_THRESHOLD = 0.02; // m/s^2
        const double GYR_THRESHOLD = 0.002; // rad/s
        bool isStatic = (accDelta < ACC_THRESHOLD) && (gyrDelta < GYR_THRESHOLD);
        ImuData filteredImuData = imuData;
        if (isStatic) {
            // 零速校正：只保留重力分量，角速度清零
            filteredImuData.acc_x = 0;
            filteredImuData.acc_y = 0;
            filteredImuData.acc_z = 9.8; // 保留重力
            filteredImuData.gyro_x = 0;
            filteredImuData.gyro_y = 0;
            filteredImuData.gyro_z = 0;
            filteredAcc = Eigen::Vector3d(0, 0, 9.8);
            filteredGyr = Eigen::Vector3d(0, 0, 0);
        } else {
            filteredImuData.acc_x = filteredAcc.x();
            filteredImuData.acc_y = filteredAcc.y();
            filteredImuData.acc_z = filteredAcc.z();
            filteredImuData.gyro_x = filteredGyr.x();
            filteredImuData.gyro_y = filteredGyr.y();
            filteredImuData.gyro_z = filteredGyr.z();
        }
        lastAcc = filteredAcc;
        lastGyr = filteredGyr;
        // --- end ---
        // 添加滤波后的IMU数据到缓冲区
        {
            QMutexLocker locker(&imuMutex);
            imuBuffer.push_back(filteredImuData);
            const size_t MAX_IMU_BUFFER_SIZE = 1000;
            if (imuBuffer.size() > MAX_IMU_BUFFER_SIZE) {
                imuBuffer.erase(imuBuffer.begin(), imuBuffer.begin() + (imuBuffer.size() - MAX_IMU_BUFFER_SIZE));
            }
        }
        // 将IMU数据传递给FAST-LIO算法处理
        fastLio->processIMU(imuData.timestamp, filteredAcc, filteredGyr);
        if (imuData.timestamp % 1000000 == 0) {
            qDebug() << "IMU数据已处理(滤波): timestamp=" << imuData.timestamp
                   << " acc=(" << filteredAcc.x() << "," << filteredAcc.y() << "," << filteredAcc.z() << ")"
                   << " gyr=(" << filteredGyr.x() << "," << filteredGyr.y() << "," << filteredGyr.z() << ")";
            double accNoiseDiff = (rawAcc - filteredAcc).norm() / (rawAcc.norm() + 1e-6) * 100.0;
            double gyrNoiseDiff = (rawGyr - filteredGyr).norm() / (rawGyr.norm() + 1e-6) * 100.0;
            qDebug() << "IMU噪声滤波效果: 加速度=" << accNoiseDiff 
                   << "%, 陀螺仪=" << gyrNoiseDiff << "%";
            if (isStatic) {
                qDebug() << "IMU静止检测: 当前为静止状态，已执行零速校正";
            }
        }
        
        // 保存静止状态供点云处理使用
        {
            QMutexLocker locker(&imuMutex);
            lastImuIsStatic = isStatic;
        }
    } catch (const std::exception& e) {
        qCritical() << "处理IMU数据异常:" << e.what();
        emit processorError(QString("处理IMU数据异常: %1").arg(e.what()));
    } catch (...) {
        qCritical() << "处理IMU数据时发生未知异常";
        emit processorError("处理IMU数据时发生未知异常");
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

// 新增：带时间戳的点云处理接口
void FastLioProcessor::processPointCloudWithTimestamp(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, uint64_t cloudTimestamp)
{
    if (!initialized || !fastLio) {
        emit processorError("Fast-LIO2未初始化，无法处理点云");
        return;
    }
    if (cloud->empty()) {
        return;
    }
    // 1. 提取IMU段
    std::vector<ImuData> imuSegment;
    {
        QMutexLocker locker(&imuMutex);
        for (const auto& imu : imuBuffer) {
            if (imu.timestamp == cloudTimestamp) { // 简化：严格等于
                imuSegment.push_back(imu);
            }
        }
    }
    if (imuSegment.empty()) {
        qWarning() << "未找到匹配的IMU数据，丢弃该帧点云 cloud_ts=" << cloudTimestamp;
        return;
    }
    // 2. 日志校验
    qDebug() << "点云帧时间戳: " << cloudTimestamp
             << " IMU区间: [" << imuSegment.front().timestamp << ", " << imuSegment.back().timestamp << "]";
    // 3. 继续原有点云处理流程
    processPointCloud(cloud);
} 