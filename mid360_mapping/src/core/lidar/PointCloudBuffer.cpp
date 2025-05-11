#include "PointCloudBuffer.h"
#include <QDebug>

PointCloudBuffer::PointCloudBuffer(QObject *parent)
    : QObject(parent)
{
}

PointCloudBuffer::~PointCloudBuffer()
{
    is_running_ = false;
    if (flush_thread_.joinable()) {
        flush_thread_.join();
    }
}

bool PointCloudBuffer::initialize(const BufferConfig& config)
{
    QMutexLocker locker(&mutex_);
    
    // 保存配置
    config_ = config;
    
    // 预分配缓冲区
    buffer_.reserve(config.max_size);
    
    // 启动刷新线程
    is_running_ = true;
    flush_thread_ = std::thread(&PointCloudBuffer::flushLoop, this);
    
    return true;
}

void PointCloudBuffer::addPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, uint64_t timestamp)
{
    QMutexLocker locker(&mutex_);
    
    // 创建新的数据包
    PointCloudPacket packet;
    packet.cloud = cloud;
    packet.timestamp = timestamp;
    packet.is_processed = false;
    
    // 添加到缓冲区
    buffer_.push_back(packet);
    
    // 如果超过最大大小，移除最旧的数据
    if (buffer_.size() > config_.max_size) {
        buffer_.removeFirst();
    }
    
    emit bufferSizeChanged(buffer_.size());
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudBuffer::getPointCloudInTimeWindow(uint64_t current_time)
{
    QMutexLocker locker(&mutex_);
    
    auto merged_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    
    // 计算时间窗口
    uint64_t window_start = current_time - static_cast<uint64_t>(config_.time_window * 1e9);
    
    // 合并时间窗口内的点云
    for (const auto& packet : buffer_) {
        if (packet.timestamp >= window_start && packet.timestamp <= current_time) {
            *merged_cloud += *packet.cloud;
        }
    }
    
    return merged_cloud;
}

void PointCloudBuffer::compress()
{
    if (!config_.enable_compression) return;
    
    QMutexLocker locker(&mutex_);
    
    // 使用体素网格进行降采样
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    voxel_grid.setLeafSize(config_.voxel_size, config_.voxel_size, config_.voxel_size);
    
    for (auto& packet : buffer_) {
        if (!packet.is_processed) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            voxel_grid.setInputCloud(packet.cloud);
            voxel_grid.filter(*filtered_cloud);
            packet.cloud = filtered_cloud;
            packet.is_processed = true;
        }
    }
    
    emit compressionCompleted();
}

void PointCloudBuffer::flush()
{
    QMutexLocker locker(&mutex_);
    
    // 移除已处理的数据
    buffer_.erase(
        std::remove_if(buffer_.begin(), buffer_.end(),
            [](const PointCloudPacket& packet) { return packet.is_processed; }),
        buffer_.end()
    );
    
    emit bufferSizeChanged(buffer_.size());
    emit flushCompleted();
}

size_t PointCloudBuffer::getBufferSize() const
{
    QMutexLocker locker(&mutex_);
    return buffer_.size();
}

PointCloudBuffer::BufferConfig PointCloudBuffer::getConfig() const
{
    QMutexLocker locker(&mutex_);
    return config_;
}

void PointCloudBuffer::updateConfig(const BufferConfig& config)
{
    QMutexLocker locker(&mutex_);
    config_ = config;
}

void PointCloudBuffer::flushLoop()
{
    while (is_running_) {
        compress();
        flush();
        std::this_thread::sleep_for(std::chrono::milliseconds(config_.flush_interval));
    }
} 