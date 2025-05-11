#include "LidarManager.h"
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDateTime>
#include <vector>
#include <chrono>
#include <cstddef> // for offsetof

static std::vector<LivoxLidarCartesianHighRawPoint> frame_buffer;
static uint8_t last_frame_cnt = 0;
static bool first_packet = true;
static const size_t MAX_POINTS_PER_FRAME = 30000; // 限制每帧最大点数
static size_t total_packets_count = 0; // 计数收到的数据包数量
static uint64_t last_publish_time = 0; // 上次发布点云的时间

LidarManager::LidarManager(QObject *parent)
    : QObject(parent)
    , connected(false)
    , scanning(false)
    , imuEnabled(false)
    , latestCloud(new pcl::PointCloud<pcl::PointXYZI>)
{
    // 创建点云缓存
    point_cloud_buffer_ = std::make_unique<PointCloudBuffer>(this);
    
    // 连接信号槽
    connect(point_cloud_buffer_.get(), &PointCloudBuffer::bufferSizeChanged,
            this, &LidarManager::onBufferSizeChanged);
    connect(point_cloud_buffer_.get(), &PointCloudBuffer::compressionCompleted,
            this, &LidarManager::onCompressionCompleted);
    connect(point_cloud_buffer_.get(), &PointCloudBuffer::flushCompleted,
            this, &LidarManager::onFlushCompleted);
}

LidarManager::~LidarManager()
{
    // 如果正在扫描，则先停止
    if (scanning) {
        stopScan();
    }
    
    // 断开连接并清理资源
    if (connected) {
        LivoxLidarSdkUninit();
    }
}

bool LidarManager::initialize(const QString &configFilePath)
{
    QMutexLocker locker(&mutex);
    
    configFile = configFilePath;
    
    // 初始化点云缓存配置
    PointCloudBuffer::BufferConfig buffer_config;
    buffer_config.max_size = 1000;        // 最大缓存1000帧
    buffer_config.flush_interval = 100;   // 每100ms刷新一次
    buffer_config.enable_compression = true;
    buffer_config.time_window = 0.1f;     // 100ms时间窗口
    buffer_config.voxel_size = 0.01f;     // 1cm体素大小
    
    if (!point_cloud_buffer_->initialize(buffer_config)) {
        emit lidarError("Failed to initialize point cloud buffer");
        return false;
    }
    
    // 初始化SDK
    if (!LivoxLidarSdkInit(configFilePath.toStdString().c_str())) {
        emit lidarError("Failed to initialize Livox SDK");
        return false;
    }
    
    // 设置回调函数
    SetLivoxLidarPointCloudCallBack(&LidarManager::onLidarDataCallback, this);
    SetLivoxLidarInfoCallback(&LidarManager::onLidarInfoCallback, this);
    SetLivoxLidarInfoChangeCallback(&LidarManager::onLidarPushCallback, this);
    
    connected = true;
    emit lidarStatusChanged(true);
    
    return true;
}

void LidarManager::startScan()
{
    QMutexLocker locker(&mutex);
    
    if (!connected || scanning) {
        return;
    }
    
    if (LivoxLidarSdkStart()) {
        scanning = true;
    } else {
        emit lidarError("Failed to start scanning");
    }
}

void LidarManager::stopScan()
{
    QMutexLocker locker(&mutex);
    
    if (!scanning) {
        return;
    }
    
    LivoxLidarSdkUninit();
    scanning = false;
    connected = false;
    emit lidarStatusChanged(false);
}

bool LidarManager::isConnected() const
{
    return connected;
}

bool LidarManager::isScanning() const
{
    return scanning;
}

bool LidarManager::isImuEnabled() const
{
    return imuEnabled;
}

void LidarManager::enableImu(bool enable)
{
    QMutexLocker locker(&mutex);
    imuEnabled = enable;
}

void LidarManager::setPointCloudBufferConfig(const PointCloudBuffer::BufferConfig& config)
{
    point_cloud_buffer_->updateConfig(config);
}

PointCloudBuffer::BufferConfig LidarManager::getPointCloudBufferConfig() const
{
    return point_cloud_buffer_->getConfig();
}

size_t LidarManager::getPointCloudBufferSize() const
{
    return point_cloud_buffer_->getBufferSize();
}

void LidarManager::processPointCloud()
{
    // 获取时间窗口内的点云数据
    auto current_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
    
    auto merged_cloud = point_cloud_buffer_->getPointCloudInTimeWindow(current_time);
    if (merged_cloud && !merged_cloud->empty()) {
        emit pointCloudReceived(merged_cloud);
        emit pointCloudWithTimestamp(merged_cloud, current_time);
    }
}

void LidarManager::onBufferSizeChanged(size_t size)
{
    emit bufferSizeChanged(size);
}

void LidarManager::onCompressionCompleted()
{
    emit compressionCompleted();
}

void LidarManager::onFlushCompleted()
{
    emit flushCompleted();
}

void LidarManager::onLidarDataCallback(const uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data)
{
    auto manager = static_cast<LidarManager*>(client_data);
    if (manager && data) {
        manager->handleLidarData(handle, reinterpret_cast<const uint8_t*>(data), data->length);
    }
}

void LidarManager::onLidarInfoCallback(const uint32_t handle, const uint8_t dev_type, const char* info, void* client_data)
{
    Q_UNUSED(dev_type);
    auto manager = static_cast<LidarManager*>(client_data);
    if (manager && info) {
        // 这里可以根据需要解析info内容
        // manager->handleLidarInfo(handle, ...);
    }
}

void LidarManager::onLidarPushCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data)
{
    auto manager = static_cast<LidarManager*>(client_data);
    if (manager && info) {
        manager->handleLidarInfo(handle, info);
    }
}

void LidarManager::handleLidarData(uint32_t handle, const uint8_t *data, uint32_t data_num)
{
    QMutexLocker locker(&mutex);
    
    // 转换点云数据
    auto cloud = convertToPointCloud(data, data_num);
    if (!cloud || cloud->empty()) {
        return;
    }
    
    // 获取当前时间戳
    auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
    
    // 添加到缓存
    point_cloud_buffer_->addPointCloud(cloud, timestamp);
    
    // 更新最新点云
    latestCloud = cloud;
}

void LidarManager::handleLidarInfo(uint32_t handle, const LivoxLidarInfo* info)
{
    if (!info) {
        return;
    }
    
    // 处理雷达信息
    // TODO: 实现雷达信息处理
}

void LidarManager::handleLidarStatus(uint32_t handle, LivoxLidarState status)
{
    QMutexLocker locker(&mutex);
    
    switch (status) {
        case kLivoxLidarStateNormal:
            connected = true;
            break;
        case kLivoxLidarStateDisconnect:
            connected = false;
            scanning = false;
            break;
        case kLivoxLidarStateError:
            connected = false;
            scanning = false;
            emit lidarError("Lidar error occurred");
            break;
        default:
            break;
    }
    
    emit lidarStatusChanged(connected);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LidarManager::convertToPointCloud(const uint8_t *data, uint32_t data_num)
{
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    
    // 添加调试输出
    qDebug() << "开始转换点云数据，数据大小:" << data_num;
    
    // 检查数据包大小
    if (data_num < sizeof(LivoxLidarEthernetPacket)) {
        qWarning() << "数据包大小太小:" << data_num << "小于最小包大小:" << sizeof(LivoxLidarEthernetPacket);
        return cloud;
    }
    
    // 解析Livox数据包
    LivoxLidarEthernetPacket* packet = reinterpret_cast<LivoxLidarEthernetPacket*>(const_cast<uint8_t*>(data));
    
    // 检查数据包类型
    qDebug() << "数据包类型:" << packet->data_type 
             << "点数量:" << packet->dot_num
             << "数据包长度:" << packet->length
             << "时间间隔:" << packet->time_interval
             << "帧计数:" << packet->frame_cnt;
    
    // 检查数据包类型
    if (packet->data_type != kLivoxLidarCartesianCoordinateHighData) {
        qWarning() << "不支持的数据类型:" << packet->data_type;
        return cloud;
    }
    
    // 检查数据包大小是否足够
    const size_t header_size = offsetof(LivoxLidarEthernetPacket, data);
    uint32_t expected_size = header_size + packet->dot_num * sizeof(LivoxLidarCartesianHighRawPoint);
    if (data_num < expected_size) {
        qWarning() << "数据包大小不足，需要:" << expected_size << "实际:" << data_num;
        return cloud;
    }
    
    // 获取点云数据
    LivoxLidarCartesianHighRawPoint* points = 
        reinterpret_cast<LivoxLidarCartesianHighRawPoint*>(packet->data);
    
    // 使用数据包中的点数量
    uint32_t point_num = packet->dot_num;
    
    qDebug() << "使用数据包中的点数量:" << point_num;
    
    // 预分配点云空间
    cloud->reserve(point_num);
    
    // 转换每个点
    for (uint32_t i = 0; i < point_num; ++i) {
        pcl::PointXYZI point;
        
        // 转换坐标（从毫米到米）
        point.x = points[i].x / 1000.0f;
        point.y = points[i].y / 1000.0f;
        point.z = points[i].z / 1000.0f;
        
        // 转换强度
        point.intensity = points[i].reflectivity;
        
        // 添加到点云
        cloud->push_back(point);
    }
    
    // 设置点云参数
    cloud->width = point_num;
    cloud->height = 1;
    cloud->is_dense = true;
    
    qDebug() << "点云转换完成，实际点数:" << cloud->size();
    
    return cloud;
}

void LidarManager::handleImuData(uint32_t handle, const uint8_t *data, uint32_t data_num)
{
    if (!imuEnabled) {
        return;
    }
    
    // 处理IMU数据
    // TODO: 实现IMU数据处理
} 